// A robot on the arena's shared Java core binding. No SDK is copied here.
// The core owns tracking, geometry, protocol parsing and act serialization.
// Bearing is earned: a scan contact's bearing is off by up to its
// bearingError (degrees, a fixed share of the arc), so a wide beam finds a
// robot and a narrow one aims at it. The robot narrows the beam, holds it
// on the target and aims along a line fitted through its recent sightings.
// Contacts within 120 units are exact.
// Clunkyard robot API 2.0.0: one core under every language, the same
// answers everywhere. Facts (a call shows its arguments):
// match: mode ruleset control robots is_3d tick_rate max_ticks
//   robot_id(slot)
// clock: tick seconds remaining_ticks
// arena: size gravity terrain_kind ground_height(x y)
// me: health position velocity heading pitch roll angular_velocity
//   attitude grounded altitude applied hull_mounts to_mount(heading pitch)
//   from_mount(heading pitch) mount_toward(point)
// battery: charge capacity regen sustainable_drive compute
//   drive_cost(fraction)
// drive: hover_lift
// gun: aim_now cooldown_ticks spread max_power damage_scale
//   can_fire(power reserve) shot_speed(power) shot_cost(power)
//   reload_ticks(power) shot_damage(power) power_for_speed(speed)
//   power_for_flight_time(range seconds) lead(contact power)
//   lead_point(position velocity power)
// scanner: last proximity_range reach(arc) arc_for_reach(range) is_on
// sensors: contacts nearest cues contact(slot) position_of(contact)
// tracker: cue nearest_bearing_rate velocity(slot) position(slot)
//   history(slot) last_seen(slot)
// nav: zone zone.margin(point) zone.contains(point margin)
//   zone.distance_from(point) storm storm.margin(point)
//   storm.contains(point margin) storm.distance_from(point) hill
//   hill.margin(point) hill.contains(point margin)
//   hill.distance_from(point) pickups nearest_pickup(kind)
//   field_distance(a b)
// score: me best armour
// angles: normalize(degrees) angle_diff(target current)
//   bearing_to(from to) elevation_to(from to) direction(heading pitch)
//   distance(a b) to_chassis(heading pitch attitude)
//   from_chassis(heading pitch attitude) clamp(value low high)
// raw: number(path) text(path) has(path) flag(path) count(path) line()
// Controls: drive.set(throttle turn pitch_turn)
//   drive.forces(thrust strafe lift yaw_torque pitch_torque roll_torque brake)
//   gun.aim(heading pitch) gun.fire(power) scanner.set(heading arc pitch)
//   scanner.power(on) tracker.configure(smoothing forget_ticks cue_ticks)
//   status.set(text value) raw.extra(json)
// Every control argument is optional; a second call keeps what the first
// set. Absent facts are your language's none. Names are snake_case here;
// your language cases them its own way. Angles are degrees from the +x
// axis; pitch is positive up.
// Manual: https://clunkyard.com/manual#sdk
import clunkyard.*
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.max
import kotlin.math.min

const val SEARCH_ARC = 40.0 // degrees: sweeps the circle in half a second
const val HOLD_WIDTH = 60.0 // units across at the target: it stays in the beam from tick to tick
const val MIN_ARC = 6.0 // degrees: the narrowest the beam holds
const val WINDOW = 12 // ticks of sightings the aim averages
const val SIGHTINGS = 8 // narrow sightings in the window before a shot
const val LOOK_EVERY = 40 // ticks between looks around the target, for a nearer robot
const val REACQUIRE = 10 // ticks to look again where a lost target was

/** A line through recent sightings: where the robot is now, how fast it moves, and how many sightings it rests on. */
class Fit(val position: Vec3, val velocity: Vec3, val count: Int)

class Bot : Robot() {
    private var dodges = 0
    private var dodgeEnd = 0
    private var slot = 0 // the last target's slot, for looking again where it was lost
    private fun steer(towards: Double, current: Double) = angles().clamp(angles().angleDiff(towards, current) / 45)

    // A straight line through the narrowest of the last WINDOW ticks' sightings: where the
    // robot is now and how fast it moves. Each sighting is off by its bearingError; the line averages that out.
    private fun fit(slot: Int, tick: Int): Fit? {
        val recent = tracker().history(slot).filter { it.tick() > tick - WINDOW }
        if (recent.size < 2) return null
        val best = recent.minOf { it.bearingError() }
        val seen = recent.filter { it.bearingError() <= 1.5 * best } // the narrow ones
        val n = seen.size
        if (n < 2) return null
        val t0 = seen.sumOf { it.tick().toDouble() } / n
        val spread = seen.sumOf { (it.tick() - t0) * (it.tick() - t0) }
        // One axis: the line's position now and its slope per tick.
        fun axis(of: (Vec3) -> Double): Pair<Double, Double> {
            val mean = seen.sumOf { of(it.position()) } / n
            val slope = seen.sumOf { (it.tick() - t0) * (of(it.position()) - mean) } / spread
            return Pair(mean + slope * (tick - t0), slope * match().tickRate())
        }
        val x = axis { it.x() }
        val y = axis { it.y() }
        val z = axis { it.z() }
        return Fit(Vec3(x.first, y.first, z.first), Vec3(x.second, y.second, z.second), n)
    }

    override fun onTurn() {
        val self = me()
        val position = self.position()
        val gun = gun()
        val scanner = scanner()
        val tick = clock().tick()
        drive().forces(Drive.brake(0.2))
        val target = sensors().nearest().orElse(null)
        val cue = tracker().cue().orElse(null)
        val lastSeen = if (slot != 0) tracker().lastSeen(slot) else java.util.OptionalInt.empty()
        val free = match().is3d() && arena().gravity().z() == 0.0
        if (target != null) {
            slot = target.slot()
            val throttle = if (target.range() > 350) 0.7 else if (target.range() < 200) -0.4 else 0.2
            drive().set(throttle, steer(target.bearing(), self.heading()), steer(target.elevation(), self.pitch()))
            // Halve the beam each tick until it is HOLD_WIDTH across at the target:
            // narrow enough for a precise bearing, wide enough to keep it in.
            val hold = max(MIN_ARC, Math.toDegrees(2 * atan2(HOLD_WIDTH / 2, target.range())))
            // In a crowd a narrow beam sees one robot. Every LOOK_EVERY ticks,
            // widen it for a tick: a nearer robot it finds becomes the target.
            val arc = if (tick % LOOK_EVERY == 0) SEARCH_ARC else max(hold, min(SEARCH_ARC, scanner.last().arc() / 2))
            scanner.set(target.bearing(), arc, target.elevation())
            val power = gun.powerForFlightTime(target.range(), 1.0)
            // An exact contact: gun.lead is the intercept if the target holds course.
            // A noisy one: lead the fitted line, and fire only once it rests on enough sightings.
            var ready = true
            val aim = if (target.bearingError() > 0) {
                val fit = fit(target.slot(), tick)
                ready = fit != null && fit.count >= SIGHTINGS
                if (fit == null) null else gun.leadPoint(fit.position, fit.velocity, power).orElse(null)
            } else gun.lead(target, power).orElse(null)
            val heading = aim?.heading() ?: target.bearing()
            val pitch = aim?.pitch() ?: target.elevation()
            gun.aim(heading, pitch)
            val now = gun.aimNow()
            if (ready && abs(angles().angleDiff(heading, now.heading())) < 3 && abs(pitch - now.pitch()) < 3 && gun.canFire(power, 10.0)) gun.fire(power)
            status().set(if (ready) "TRACK" else "NARROW", target.range())
        } else if (lastSeen.isPresent && tick - lastSeen.asInt <= REACQUIRE) {
            // Lost it: look again where it was last seen, widening the beam, before searching.
            val where = tracker().position(slot).orElse(position)
            val bearing = angles().bearingTo(position, where)
            val elevation = if (free) angles().elevationTo(position, where) else 0.0
            drive().set(0.5, steer(bearing, self.heading()), steer(elevation, self.pitch()))
            scanner.set(bearing, min(SEARCH_ARC, scanner.last().arc() * 2), elevation)
            status().set("REACQUIRE")
        } else if (cue != null) {
            val bearing = angles().bearingTo(position, cue.position())
            val distance = angles().distance(position, cue.position())
            val elevation = if (free) angles().elevationTo(position, cue.position()) else 0.0
            val error = min(cue.bearingError(), 30.0)
            val across = (tick % 5 - 2) / 2.0 * error
            val up = if (free) ((tick / 5) % 3 - 1) * error / 2 else 0.0
            drive().set(0.7, steer(bearing, self.heading()), steer(elevation, self.pitch()))
            scanner.set(bearing + across, scanner.arcForReach(distance + 100), elevation + up)
            status().set("CUE", distance)
        } else {
            drive().set(0.5, 0.33, steer(0.0, self.pitch()))
            val pitch = if (free) ((tick / 10) % 3 - 1) * 35.0 else 0.0
            scanner.set(angles().normalize(scanner.last().heading() + 36), SEARCH_ARC, pitch)
            status().set("SEARCH")
        }
        // A simple dodge: vary it, or predict the shooter.
        if (tick >= dodgeEnd && sensors().cues().any { it.source() == Source.MUZZLE }) {
            dodges++
            dodgeEnd = tick + 10
        }
        if (tick < dodgeEnd) {
            // Off the line of fire: left, then right next time (turn is positive left, strafe positive right).
            val left = dodges % 2 == 1
            if (match().control() == Control.FORCES) {
                drive().set(Drive.throttle(0.5))
                drive().forces(Drive.strafe(if (left) -1.0 else 1.0))
            } else drive().set(1.0, if (left) 1.0 else -1.0)
            status().set(if (left) "dodge:left" else "dodge:right", dodges.toDouble())
        }
        if (battery().charge() < battery().capacity() / 4) drive().set(Drive.throttle(drive().get("throttle").orElse(0.0) * 0.5))
        val storm = nav().storm().orElse(null)
        if (storm != null && storm.margin(position).orElse(-1e9) > -60) {
            val away = angles().bearingTo(storm.center(), position)
            drive().set(Drive.throttle(0.8).turn(angles().clamp(angles().angleDiff(away, self.heading()) / 30)))
        }
        val zone = nav().zone().orElse(null)
        if (zone != null && zone.margin(position).orElse(1e9) < 60) {
            val climb = if (zone.shape() == Shape.SPHERE) angles().elevationTo(position, zone.center()) else 0.0
            drive().set(0.8, angles().clamp(angles().angleDiff(angles().bearingTo(position, zone.center()), self.heading()) / 30), angles().clamp((climb - self.pitch()) / 30))
        }
    }
}


fun main() = Clunkyard.run(Bot())

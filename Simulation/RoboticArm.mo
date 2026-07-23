package RoboticArm "2-DOF desktop arm: shoulder + elbow pick-and-place.

   This version focuses on the two most risk-relevant joints (shoulder and
   elbow). The structure is intentionally simple and easy to extend later by
   adding more servo blocks, waypoints, and plant equations.

   Angles [deg], absolute from horizontal, CCW+:
     shoulder / elbow — arm pose in the vertical plane

   Event-driven waypoints: advance when the shoulder and elbow have settled.
   Each active joint is a stall-limited PD servo; gravity on shoulder/elbow
   uses the CAD lump masses.

   Edit: _servo_shoulder / _servo_elbow, _path_input, _mechanics_input.
  "
  // ===========================================================================
  // Inputs
  // ===========================================================================

  record ServoInput "One joint drive: datasheet-style stall torque + no-load speed"
    import Modelica.Units.SI;
    SI.Torque tau_stall = 0.44 "Stall torque [N·m]";
    Real max_speed(unit = "deg/s") = 394 "No-load speed [deg/s]";
    SI.Mass servo_mass = 0.055 "Servo package mass [kg]";
  end ServoInput;

  record ShoulderJointInput "Shoulder-side joint and link definition"
    import Modelica.Units.SI;
    Real position_sequence[7](each unit = "deg") = {70, 50, 50, 75, 55, 55, 70} "Shoulder waypoint positions [deg]";
    ServoInput servo "Shoulder servo configuration";
    SI.Length link_length = 0.220 "Shoulder-to-elbow length [m]";
    SI.Mass link_mass = 0.04 "Upper-arm mass [kg]";
    constant SI.Inertia inertia = 0.006 "Best-guess shoulder inertia [kg·m²]";
    constant Real viscous_friction(unit = "N.m.s/rad") = 0.05 "Best-guess shoulder viscous friction [N·m·s/rad]";
  end ShoulderJointInput;

  record ElbowJointInput "Elbow-side joint and link definition"
    import Modelica.Units.SI;
    Real position_sequence[7](each unit = "deg") = {-45, -30, -30, 5, -25, -25, -45} "Elbow waypoint positions [deg]";
    ServoInput servo "Elbow servo configuration";
    SI.Length link_length = 0.120 "Elbow-to-tool length [m]";
    SI.Mass link_mass = 0.045 "Forearm mass [kg]";
    constant SI.Inertia inertia = 0.0015 "Best-guess elbow inertia [kg·m²]";
    constant Real viscous_friction(unit = "N.m.s/rad") = 0.05 "Best-guess elbow viscous friction [N·m·s/rad]";
  end ElbowJointInput;

  record LoadInput "Payload and tool proxy for the arm tip"
    import Modelica.Units.SI;
    SI.Mass proxy_mass = 0.01 "Proxy mass at the arm tip [kg]";
    SI.Length proxy_length = 0.080 "Proxy length along the forearm [m]";
    SI.Length proxy_z_offset = 0.040 "Proxy vertical offset [m]";
    Boolean carry_sequence[6] = {false, true, true, true, false, false} "Carry-load flag for each waypoint stage";
  end LoadInput;

  // ===========================================================================
  // Plant
  // ===========================================================================

  model PositionServo "Speed- and torque-limited servo; command in deg.

                     Cascaded saturated-P control (position -> velocity -> torque), each
                     stage clamped to the servo's actual rating (max_speed, tau_stall) so
                     those two datasheet numbers are what determines the motion, not a
                     tuned gain. position_loop_gain/velocity_loop_gain are just internal
                     loop-bandwidth constants, picked large enough that the outer limits
                     are what bind in practice."
    import Modelica.Units.SI;
    import Modelica.Units.Conversions.from_deg;
    import Modelica.Constants.pi;
    parameter SI.Torque tau_stall = 0.92 annotation(
      HideResult = true);
    parameter Real max_speed(unit = "deg/s") = 300 annotation(
      HideResult = true);
    Modelica.Blocks.Interfaces.RealInput position_command(unit = "deg") annotation(
      HideResult = true);
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange annotation(
      HideResult = true);
    SI.Torque torque_output annotation(
      HideResult = true);
  protected
    constant Real position_loop_gain(unit = "1/s") = 10 "Position-loop bandwidth (not a servo spec)";
    constant Real velocity_loop_gain(unit = "N.m.s/rad") = 50 "Velocity-loop gain (not a servo spec)";
    SI.AngularVelocity max_angular_speed = max_speed*pi/180;
    SI.Angle joint_angle = flange.phi;
    SI.AngularVelocity joint_angular_velocity = der(joint_angle);
    SI.Angle position_error = from_deg(position_command) - joint_angle;
    SI.AngularVelocity velocity_command = smooth(0, noEvent(if position_loop_gain*position_error > max_angular_speed then max_angular_speed elseif position_loop_gain*position_error < -max_angular_speed then -max_angular_speed else position_loop_gain*position_error));
    SI.Torque torque_command = velocity_loop_gain*(velocity_command - joint_angular_velocity);
  equation
    torque_output = smooth(0, noEvent(if torque_command > tau_stall then tau_stall elseif torque_command < -tau_stall then -tau_stall else torque_command));
    flange.tau = -torque_output;
  end PositionServo;

  model ArmPlant "Planar 2-link arm with a placeholder for future optional joints."
    import Modelica.Units.SI;
    import Modelica.Constants.g_n;
    import Modelica.Units.Conversions.from_deg;
    parameter ShoulderJointInput shoulder_input annotation(
      HideResult = true);
    parameter ElbowJointInput elbow_input annotation(
      HideResult = true);
    parameter LoadInput load_input annotation(
      HideResult = true);
    parameter Real shoulder_angle_0(unit = "deg") = 70 annotation(
      HideResult = true);
    parameter Real elbow_angle_0(unit = "deg") = -45 annotation(
      HideResult = true);
    Modelica.Blocks.Interfaces.RealInput payload_mass(unit = "kg") annotation(
      HideResult = true);
    Modelica.Blocks.Interfaces.BooleanInput carrying annotation(
      HideResult = true);
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_shoulder annotation(
      HideResult = true);
    Modelica.Mechanics.Rotational.Interfaces.Flange_a flange_elbow annotation(
      HideResult = true);
    SI.Angle shoulder_angle(start = from_deg(shoulder_angle_0), fixed = true) annotation(
      HideResult = true);
    SI.Angle elbow_angle(start = from_deg(elbow_angle_0), fixed = true) annotation(
      HideResult = true);
    SI.AngularVelocity shoulder_angular_velocity(start = 0, fixed = true) annotation(
      HideResult = true);
    SI.AngularVelocity elbow_angular_velocity(start = 0, fixed = true) annotation(
      HideResult = true);
    SI.Torque gravity_torque_shoulder annotation(
      HideResult = true);
    SI.Torque gravity_torque_elbow annotation(
      HideResult = true);
  protected
    SI.Length elbow_joint_x = shoulder_input.link_length*cos(shoulder_angle);
    SI.Length forearm_com_x = elbow_joint_x + (elbow_input.link_length/2)*cos(elbow_angle);
    SI.Length forearm_tip_x = elbow_joint_x + elbow_input.link_length*cos(elbow_angle);
    // forearm-frame (x,z) → horizontal lever at elbow_angle; shared by the tool
    // proxy and any carried payload since both sit at the same tool-tip point.
    SI.Length tool_tip_x = forearm_tip_x + load_input.proxy_length*cos(elbow_angle) - load_input.proxy_z_offset*sin(elbow_angle);
    SI.Inertia shoulder_inertia = shoulder_input.inertia + shoulder_input.link_mass*(shoulder_input.link_length/2)^2 + shoulder_input.servo.servo_mass*shoulder_input.link_length^2 + elbow_input.servo.servo_mass*(shoulder_input.link_length + elbow_input.link_length)^2 + elbow_input.link_mass*(shoulder_input.link_length + elbow_input.link_length/2)^2 + load_input.proxy_mass*(shoulder_input.link_length + elbow_input.link_length + load_input.proxy_length)^2 + payload_mass*(shoulder_input.link_length + elbow_input.link_length + load_input.proxy_length)^2;
    SI.Inertia elbow_inertia = elbow_input.inertia + elbow_input.link_mass*(elbow_input.link_length/2)^2 + elbow_input.servo.servo_mass*elbow_input.link_length^2 + load_input.proxy_mass*(elbow_input.link_length + load_input.proxy_length)^2 + payload_mass*(elbow_input.link_length + load_input.proxy_length)^2;
  equation
    shoulder_angle = flange_shoulder.phi;
    elbow_angle = flange_elbow.phi;
    shoulder_angular_velocity = der(shoulder_angle);
    elbow_angular_velocity = der(elbow_angle);
// elbow servo mass sits at the tip of the elbow's own link (forearm_tip_x),
// same convention as the shoulder servo mass sitting at the tip of its link
// (elbow_joint_x) -- one link further down the chain.
    gravity_torque_shoulder = g_n*(shoulder_input.link_mass*(shoulder_input.link_length/2)*cos(shoulder_angle) + shoulder_input.servo.servo_mass*elbow_joint_x + elbow_input.servo.servo_mass*forearm_tip_x + elbow_input.link_mass*forearm_com_x + load_input.proxy_mass*tool_tip_x + payload_mass*tool_tip_x);
    gravity_torque_elbow = g_n*(elbow_input.link_mass*(forearm_com_x - elbow_joint_x) + elbow_input.servo.servo_mass*(forearm_tip_x - elbow_joint_x) + load_input.proxy_mass*(tool_tip_x - elbow_joint_x) + payload_mass*(tool_tip_x - elbow_joint_x));
    shoulder_inertia*der(shoulder_angular_velocity) = flange_shoulder.tau - gravity_torque_shoulder - shoulder_input.viscous_friction*shoulder_angular_velocity;
    elbow_inertia*der(elbow_angular_velocity) = flange_elbow.tau - gravity_torque_elbow - elbow_input.viscous_friction*elbow_angular_velocity;
  end ArmPlant;

  // ===========================================================================
  // Experiment
  // ===========================================================================

  model RoboticArmSimulation "Feasible pick-and-place; results expose move_time for servo sweeps."
    import Modelica.Units.SI;
    import Modelica.Constants.pi;
    // Joint inputs use their record defaults as-is (already the intended
    // shoulder/elbow configuration) -- no per-run override scaffolding.
    parameter ShoulderJointInput _shoulder_input "Shoulder joint input" annotation(
      Dialog);
    parameter ElbowJointInput _elbow_input "Elbow joint input" annotation(
      Dialog);
    parameter LoadInput _load_input annotation(
      Dialog);
    PositionServo servo_sh(tau_stall = _shoulder_input.servo.tau_stall, max_speed = _shoulder_input.servo.max_speed) annotation(
      HideResult = true);
    PositionServo servo_el(tau_stall = _elbow_input.servo.tau_stall, max_speed = _elbow_input.servo.max_speed) annotation(
      HideResult = true);
    ArmPlant plant(shoulder_input = _shoulder_input, elbow_input = _elbow_input, load_input = _load_input, shoulder_angle_0 = _shoulder_input.position_sequence[1], elbow_angle_0 = _elbow_input.position_sequence[1]) annotation(
      HideResult = true);
    // --- plot / harvest ------------------------------------------------------
    Real shoulder_setpoint(unit = "deg") "Shoulder command [deg]";
    Real elbow_setpoint(unit = "deg") "Elbow command [deg]";
    Real shoulder_actual_angle(unit = "deg") "Shoulder actual [deg]";
    Real elbow_actual_angle(unit = "deg") "Elbow actual [deg]";
    SI.Torque shoulder_torque "Shoulder servo torque [N·m]";
    SI.Torque elbow_torque "Elbow servo torque [N·m]";
    // 'discrete' matters: continuous variables assigned in an algorithm are
    // re-initialized from their start value on every invocation and would
    // lose the timestamp; discrete ones hold their pre() value.
    discrete SI.Time move_time(start = 0, fixed = true) "Seconds when the last waypoint was reached (0 if unfinished)";
  protected
    Integer current_stage(start = 1, fixed = true) "1..6 → p1..p6; 7 = done";
    Boolean carrying(start = _load_input.carry_sequence[1], fixed = true);
    SI.Mass payload_mass;
    Real shoulder_target(unit = "deg");
    Real elbow_target(unit = "deg");
    Boolean shoulder_at_target;
    Boolean elbow_at_target;
    Boolean shoulder_settled;
    Boolean elbow_settled;
    Boolean arrived;

    function twoJointWaypointSelect
      input Integer stage;
      input ShoulderJointInput shoulder_input;
      input ElbowJointInput elbow_input;
      output Real shoulder_target;
      output Real elbow_target;
    algorithm
      if stage <= 1 then
        shoulder_target := shoulder_input.position_sequence[2];
        elbow_target := elbow_input.position_sequence[2];
      elseif stage == 2 then
        shoulder_target := shoulder_input.position_sequence[3];
        elbow_target := elbow_input.position_sequence[3];
      elseif stage == 3 then
        shoulder_target := shoulder_input.position_sequence[4];
        elbow_target := elbow_input.position_sequence[4];
      elseif stage == 4 then
        shoulder_target := shoulder_input.position_sequence[5];
        elbow_target := elbow_input.position_sequence[5];
      elseif stage == 5 then
        shoulder_target := shoulder_input.position_sequence[6];
        elbow_target := elbow_input.position_sequence[6];
      else
        shoulder_target := shoulder_input.position_sequence[7];
        elbow_target := elbow_input.position_sequence[7];
      end if;
    end twoJointWaypointSelect;
  equation
    connect(servo_sh.flange, plant.flange_shoulder);
    connect(servo_el.flange, plant.flange_elbow);
    servo_sh.position_command = shoulder_setpoint;
    servo_el.position_command = elbow_setpoint;
    plant.payload_mass = payload_mass;
    plant.carrying = carrying;
    payload_mass = if carrying then _load_input.proxy_mass else 0;
// pre() breaks the algebraic event loop stage -> target -> arrived -> stage,
// which OpenModelica would otherwise turn into an (unsupported) mixed
// nonlinear system.
    (shoulder_target, elbow_target) = twoJointWaypointSelect(pre(current_stage), _shoulder_input, _elbow_input);
// Servos are now speed-limited themselves (max_speed), so the waypoint
// target can be commanded directly -- no need for a separate slew-rate
// pre-filter ahead of the servo.
    shoulder_setpoint = shoulder_target;
    elbow_setpoint = elbow_target;
    shoulder_actual_angle = (180/pi)*plant.shoulder_angle;
    elbow_actual_angle = (180/pi)*plant.elbow_angle;
    shoulder_torque = servo_sh.torque_output;
    elbow_torque = servo_el.torque_output;
    shoulder_at_target = abs(shoulder_actual_angle - shoulder_target) < 2.5;
    elbow_at_target = abs(elbow_actual_angle - elbow_target) < 2.5;
    shoulder_settled = abs(der(shoulder_actual_angle)) < 12.0;
    elbow_settled = abs(der(elbow_actual_angle)) < 12.0;
    arrived = shoulder_at_target and elbow_at_target and shoulder_settled and elbow_settled;
  algorithm
// Sampled stage machine (one advance per tick). Edge-triggered
// when-clauses on 'arrived' would deadlock on identical consecutive
// waypoints (no new rising edge) and drag the discrete loop into a
// nonlinear system OpenModelica cannot lower.
    when sample(0.02, 0.02) then
      if arrived and current_stage < 7 then
        if current_stage == 6 then
          move_time := time;
        end if;
        carrying := _load_input.carry_sequence[current_stage];
        current_stage := current_stage + 1;
      end if;
    end when;
    annotation(
      experiment(StartTime = 0, StopTime = 15, Tolerance = 1e-6, Interval = 0.02));
  end RoboticArmSimulation;
  annotation(
    uses(Modelica(version = "4.0.0")));
end RoboticArm;

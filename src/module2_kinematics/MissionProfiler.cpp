#include "../../include/module2_kinematics/MissionProfiler.hpp"
#include "../../include/roles/IPayloadRole.hpp"
#include "../../include/constants/PhysicalConstants.hpp"
#include <cmath>

namespace Module2 {

    /**
     * Constructor initializing the mission profiler evaluator.
     * Caches atmospheric air density, user sizing constraints, and active role descriptors
     * to resolve vehicle kinematics across the climb, forward cruise, and hover phases.
     */
    MissionEvaluator::MissionEvaluator(double air_density, const EngineData::UserInput& input, const EngineData::IPayloadRole* role) 
        : aero_overrides(input.aero_overrides)
    {
        this->rho = air_density;
        this->v_forward = input.v_forward_ms;
        this->v_climb = input.v_climb_ms;
        this->t_hover = input.t_hover_s;
        this->t_climb = input.t_climb_s;
        this->t_forward = input.t_forward_s;
        this->active_role = role;
    }

    /**
     * Calculates the aerodynamic body parasite drag force acting on the drone fuselage.
     * 
     * Aerodynamic & Geometric Scaling:
     * - Fuselage equivalent flat-plate area is scaled dynamically relative to the 2/3rds power 
     *   of total vehicle mass. This reflects structural scaling laws where surface area scales 
     *   proportionally with volume^2/3 (dimensional scaling).
     * - Role-Specific Drag: Active payload roles (e.g. gimbal cams, agricultural spray booms) 
     *   append an additional drag area penalty (getAddedDragAreaM2) representing component bluff-body drag.
     * - Drag Equation: Drag = 0.5 * rho * V^2 * Cd * Area
     */
    double MissionEvaluator::calculateParasiteDrag(double mass, double velocity, double drag_coefficient) const {
        double scaling_factor = (aero_overrides.area_scaling_factor > 0.0) ? aero_overrides.area_scaling_factor : 0.05;
        double equivalent_area = scaling_factor * std::pow(mass, 2.0 / 3.0);
        if (this->active_role != nullptr) {
            equivalent_area += this->active_role->getAddedDragAreaM2();
        }
        // Drag = 0.5 * rho * V^2 * Cd * A
        double drag_force = 0.5 * rho * std::pow(velocity, 2) * drag_coefficient * equivalent_area;
        return drag_force;
    }

    /**
     * Resolves flight characteristics during steady hover.
     * 
     * Kinematic Resolution (PROFILER-01):
     * - In a zero-velocity hover, the fuselage pitch angle is zero.
     * - Thrust must balance gravity.
     * - Agility Scaling: High-agility roles (e.g. racing, bridge inspection) make aggressive, 
     *   high-frequency attitude adjustments. To prevent voltage sag and ensure maneuverability headroom, 
     *   we pad the effective hover thrust target by 25% if the role demands high margin reserves.
     */
    EngineData::MissionPhase MissionEvaluator::getHoverState(double current_mass) const {
        EngineData::MissionPhase phase;
        phase.duration_s = this->t_hover;
        phase.velocity_ms = 0.0;
        phase.pitch_angle_rad = 0.0; 
        
        // --- PROFILER-01 FIX: Account for role-specific agility margins in hover ---
        double thrust_mult = 1.0;
        if (active_role != nullptr && active_role->getRoleType() == "racing") {
            // Only racing drones require padded nominal hover thrust for high-frequency control sag
            thrust_mult = 1.25;
        }
        phase.thrust_req_n = current_mass * Physics::GRAVITY_MS2 * thrust_mult;
        return phase;
    }

    /**
     * Resolves flight characteristics during vertical climb.
     * 
     * Force Balance:
     * - Fuselage pitch angle remains zero.
     * - Required thrust must balance both vehicle takeoff weight and vertical body parasite drag: 
     *   T = Weight + Drag_vertical
     * - Vertical drag uses a high baseline Cd (default 1.4) representing the bluff-body 
     *   aerodynamics of a copter climbing flat against the wind.
     */
    EngineData::MissionPhase MissionEvaluator::getClimbState(double current_mass) const {
        EngineData::MissionPhase phase;
        phase.duration_s = this->t_climb;
        phase.velocity_ms = this->v_climb;
        phase.pitch_angle_rad = 0.0; 
        double weight = current_mass * Physics::GRAVITY_MS2;
        double cd_vertical = (aero_overrides.cd_vertical > 0.0) ? aero_overrides.cd_vertical : 1.4;
        double drag = calculateParasiteDrag(current_mass, this->v_climb, cd_vertical);
        phase.thrust_req_n = weight + drag;
        return phase;
    }

    /**
     * Resolves flight characteristics during steady forward flight.
     * 
     * Vector Force Resolution:
     * - Multicopters generate forward propulsive force by pitching the airframe forward.
     * - Forward Pitch Angle (theta) is solved via force balance: tan(theta) = Drag / Weight.
     * - Required Thrust is the vector sum of vertical lift (balancing gravity) and horizontal 
     *   propulsive force (balancing body parasite drag):
     *   T = sqrt(Weight^2 + Drag_horizontal^2)
     * - Horizontal drag uses a standard Cd (default 1.1) representing the forward fuselage profile.
     */
    EngineData::MissionPhase MissionEvaluator::getForwardState(double current_mass) const {
        EngineData::MissionPhase phase;
        phase.duration_s = this->t_forward;
        phase.velocity_ms = this->v_forward;
        double weight = current_mass * Physics::GRAVITY_MS2;
        double cd_horizontal = (aero_overrides.cd_horizontal > 0.0) ? aero_overrides.cd_horizontal : 1.1;
        double drag = calculateParasiteDrag(current_mass, this->v_forward, cd_horizontal);
        phase.thrust_req_n = std::sqrt(std::pow(weight, 2) + std::pow(drag, 2));
        phase.pitch_angle_rad = std::atan2(drag, weight);
        return phase;
    }

}
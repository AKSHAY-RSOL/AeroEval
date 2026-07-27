#ifndef I_PAYLOAD_ROLE_HPP
#define I_PAYLOAD_ROLE_HPP

#include <string>

namespace EngineData {

    // Interface for payload roles customizing drone physical simulation behavior.
    class IPayloadRole {
    public:
        virtual ~IPayloadRole() = default;

        // Auxiliary electrical power draw (W) from camera, spray pumps, etc.
        virtual double getAuxiliaryPowerW() const = 0; 
        
        // Frontal drag area (m^2) added by payload.
        virtual double getAddedDragAreaM2() const = 0; 
        
        // Asymmetric CG offset (m) along arms.
        virtual double getCenterOfMassShiftM() const = 0; 
        
        // Required motor control head-room thrust margin (0 to 1).
        virtual double getRequiredThrustMargin() const = 0; 
        
        // Structural load factor (G-load) during maneuvers.
        virtual double getDynamicLoadFactor() const { return 1.5; }
        
        // Maximum downwash velocity (m/s) allowed under rotors.
        virtual double getMaxInducedVelocityMs() const { return 25.0; }
        
        // Packaged mass dropped mid-mission (kg).
        virtual double getDiscreteMassSheddingKg() const { return 0.0; }

        // Time fraction of mission when package is dropped (e.g., 0.5 = halfway).
        virtual double getDropTimeRatio() const { return 0.5; }
        
        // Liquid spray depletion rate (kg/s).
        virtual double getContinuousMassSheddingRateKgPerS(double flight_time_multiplier = 1.0) const { return 0.0; }

        // Identify role classification type
        virtual std::string getRoleType() const { return "imaging"; }
    };

}

#endif // I_PAYLOAD_ROLE_HPP
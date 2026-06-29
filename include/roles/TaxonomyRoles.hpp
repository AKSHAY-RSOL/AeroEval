#ifndef TAXONOMY_ROLES_HPP
#define TAXONOMY_ROLES_HPP

#include "../utils/DataStructures.hpp"
#include "IPayloadRole.hpp"


namespace EngineData {

// Cinematography / Imaging Role.
// Enforces 55% thrust margin for control authority in high-precision filming.
class CinematographyRole : public IPayloadRole {
private:
  PayloadRoleConfig config;

public:
  CinematographyRole(const PayloadRoleConfig &c) : config(c) {}
  double getAuxiliaryPowerW() const override { return config.aux_power_w; }
  double getAddedDragAreaM2() const override { return config.added_drag_m2; }
  double getCenterOfMassShiftM() const override { return config.cg_shift_m; }
  double getRequiredThrustMargin() const override {
    return config.thrust_margin_override > 0 ? config.thrust_margin_override
                                             : 0.45;
  }
  std::string getRoleType() const override { return "imaging"; }
};

// Delivery & Logistics Role.
// Models discrete cargo drop mid-mission. Saves battery for the return trip.
class DeliveryRole : public IPayloadRole {
private:
  PayloadRoleConfig config;
  double package_mass_kg;

public:
  DeliveryRole(const PayloadRoleConfig &c, double package_mass)
      : config(c), package_mass_kg(package_mass) {}
  double getAuxiliaryPowerW() const override { return config.aux_power_w; }
  double getAddedDragAreaM2() const override { return config.added_drag_m2; }
  double getCenterOfMassShiftM() const override { return config.cg_shift_m; }
  double getRequiredThrustMargin() const override {
    return config.thrust_margin_override > 0 ? config.thrust_margin_override
                                             : 0.40;
  }

  double getDiscreteMassSheddingKg() const override {
    return config.delivery_drop_mass_kg >= 0 ? config.delivery_drop_mass_kg
                                             : package_mass_kg;
  }
  double getDropTimeRatio() const override {
    return config.delivery_drop_time_ratio >= 0
               ? config.delivery_drop_time_ratio
               : 0.5;
  }
  std::string getRoleType() const override { return "delivery"; }
};

// Agriculture (Crop Spraying) Role.
// Models continuous liquid payload discharge and strict downwash velocity
// limits.
class AgricultureRole : public IPayloadRole {
private:
  PayloadRoleConfig config;
  double spray_fluid_mass_kg;
  double total_flight_time_s;

public:
  AgricultureRole(const PayloadRoleConfig &c, double fluid_mass,
                  double flight_time)
      : config(c), spray_fluid_mass_kg(fluid_mass),
        total_flight_time_s(flight_time) {}
  double getAuxiliaryPowerW() const override { return config.aux_power_w; }
  double getAddedDragAreaM2() const override { return config.added_drag_m2; }
  double getCenterOfMassShiftM() const override { return config.cg_shift_m; }
  double getRequiredThrustMargin() const override {
    return config.thrust_margin_override > 0 ? config.thrust_margin_override
                                             : 0.40;
  }

  double getMaxInducedVelocityMs() const override {
    return config.ag_max_downwash_ms > 0 ? config.ag_max_downwash_ms : 25.0;
  }
  double getContinuousMassSheddingRateKgPerS(double flight_time_multiplier = 1.0) const override {
    if (config.spray_rate_kg_per_s >= 0)
      return config.spray_rate_kg_per_s;
    double current_flight_time = total_flight_time_s * flight_time_multiplier;
    if (current_flight_time <= 0)
      return 0;
    return spray_fluid_mass_kg / current_flight_time;
  }
  std::string getRoleType() const override { return "agriculture"; }
};

// Mapping / Photogrammetry Role.
// Accounts for bulky mapping payloads with default 1.5x drag multiplier.
class MappingRole : public IPayloadRole {
private:
  PayloadRoleConfig config;

public:
  MappingRole(const PayloadRoleConfig &c) : config(c) {}
  double getAuxiliaryPowerW() const override { return config.aux_power_w; }
  double getAddedDragAreaM2() const override {
    double multiplier =
        (config.drag_area_multiplier > 0.0) ? config.drag_area_multiplier : 1.5;
    return config.added_drag_m2 * multiplier;
  }
  double getCenterOfMassShiftM() const override { return config.cg_shift_m; }
  double getRequiredThrustMargin() const override {
    return config.thrust_margin_override > 0 ? config.thrust_margin_override
                                             : 0.40;
  }
  std::string getRoleType() const override { return "mapping"; }
};

// Racing / Acrobatic (FPV) Role.
// Optimizes agility by requiring 80% thrust margin and standing up to 10G
// loads.
class RacingRole : public IPayloadRole {
private:
  PayloadRoleConfig config;

public:
  RacingRole(const PayloadRoleConfig &c) : config(c) {}
  double getAuxiliaryPowerW() const override { return config.aux_power_w; }
  double getAddedDragAreaM2() const override { return config.added_drag_m2; }
  double getCenterOfMassShiftM() const override { return config.cg_shift_m; }
  double getRequiredThrustMargin() const override {
    return config.thrust_margin_override > 0 ? config.thrust_margin_override
                                             : 0.80;
  }

  double getDynamicLoadFactor() const override {
    return config.racing_load_factor > 0 ? config.racing_load_factor : 10.0;
  }
  double getMaxInducedVelocityMs() const override {
    // ROLE-02 fix: Racing drones are not constrained by VRS or agricultural 
    // downwash limits. A high limit avoids artificial engine bottlenecks here.
    return 100.0; 
  }
  std::string getRoleType() const override { return "racing"; }
};

// Industrial Inspection Role.
// Designed for close proximity flights. Demands 70% thrust margin.
class InspectionRole : public IPayloadRole {
private:
  PayloadRoleConfig config;

public:
  InspectionRole(const PayloadRoleConfig &c) : config(c) {}
  double getAuxiliaryPowerW() const override { return config.aux_power_w; }
  double getAddedDragAreaM2() const override { return config.added_drag_m2; }
  double getCenterOfMassShiftM() const override { return config.cg_shift_m; }
  double getRequiredThrustMargin() const override {
    return config.thrust_margin_override > 0 ? config.thrust_margin_override
                                             : 0.50;
  }
  std::string getRoleType() const override { return "inspection"; }
};

} // namespace EngineData

#endif // TAXONOMY_ROLES_HPP

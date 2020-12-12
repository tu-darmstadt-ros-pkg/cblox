#ifndef CBLOX_INTEGRATOR_GENERIC_SUBMAP_COLLECTION_INTEGRATOR_H_
#define CBLOX_INTEGRATOR_GENERIC_SUBMAP_COLLECTION_INTEGRATOR_H_

#include "cblox/core/common.h"
#include "cblox/core/generic_submap.h"
#include "cblox/core/generic_submap_collection.h"

// Generic Version of the tsdf_submap collection integrator
namespace cblox {

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
class GenericSubmapCollectionIntegrator {
 public:
  GenericSubmapCollectionIntegrator(
      const std::shared_ptr<GenericSubmapCollection<VoxelType>>&
          submap_collection_ptr,
      const typename IntegratorType::ConfigType config)
      : submap_collection_ptr_(submap_collection_ptr), config_(config) {}
  // TODO Integrator base??, maybe change interface??

  void integrate(const Transformation& T_G_C, const IntegrationData& data);

  // Changes the active submap to the last one on the collection
  void switchToActiveSubmap();

  void setIntegrator(std::shared_ptr<IntegratorType> integ);

 protected:
  void initializeIntegrator();

  void updateIntegratorTarget();

  Transformation getSubmapRelativePose(const Transformation& T_G_C) const;

  std::shared_ptr<GenericSubmapCollection<VoxelType>> submap_collection_ptr_;

  Transformation T_G_S_active_;

  std::shared_ptr<IntegratorType> integrator_;

  void (*integration_function_)(VoxelType&, IntegrationData&);

  typename IntegratorType::ConfigType config_;
};

}  // namespace cblox

#endif  // CBLOX_INTEGRATOR_GENERIC_SUBMAP_COLLECTION_INTEGRATOR_H_
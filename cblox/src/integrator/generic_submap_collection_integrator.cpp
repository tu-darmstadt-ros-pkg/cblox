#include "cblox/integrator/generic_submap_collection_integrator.h"
#include "cblox/core/generic_submap.h"

#include "voxblox/integrator/tsdf_integrator.h"

namespace cblox {

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<
    IntegratorType, IntegrationData,
    VoxelType>::integrate(const Transformation& T_G_C,
                          const IntegrationData& data) {
  CHECK(!submap_collection_ptr_->empty())
      << "Can't integrate. No submaps in collection.";
  CHECK(integrator_) << "Can't integrate. Need to update integration target.";

  const Transformation T_S_C = getSubmapRelativePose(T_G_C);

  // TODO rethink mutex

  // std::cout << "general integrating" << std::endl;
  integrator_->integrate(T_S_C, data);
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData,
                                       VoxelType>::switchToActiveSubmap() {
  updateIntegratorTarget();
      //submap_collection_ptr_->getActiveSubmapPtr()->getMapPtr());
  T_G_S_active_ = submap_collection_ptr_->getActiveSubmapPose();
  // TODO check if this can be done with a simple layer??, maybe reuse tsdf map?
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData,
                                       VoxelType>::
    initializeIntegrator() {
  //CHECK(map_ptr);
  // TODO, create integrator here
  // void (*integration_function)(VoxelType&, IntegrationData&);
  // or maybe try to declare them explicit instead of passing?

  // TODO config passing in this class is needed

  // or integrator must be instantiated elsewhere

  // TODO pass everything in config

  //Layer<VoxelType>* layer = new Layer<VoxelType>(0.2, 16u);
  //voxblox::TsdfIntegratorBase::Config config;
  //config.default_truncation_distance = 0.4;
  // integrator_.reset(new IntegratorType(NULL, &layer, integration_function));
  // integrator_.reset(new IntegratorType(config, layer));
  // TODO repair this with use of config structs!!!

  integrator_.reset(new IntegratorType(config_));
}

//TODO removed option to go to older submaps, currently only using newest ones. check if that deature is needed
template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData,
                                       VoxelType>::
    updateIntegratorTarget() {
  //CHECK(map_ptr);

  if (integrator_ == nullptr) {
    initializeIntegrator();
    integrator_->setActiveLayers();
  } else {
    std::cout << "updated integrator target" << std::endl;
    integrator_->setActiveLayers();
  }
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
Transformation GenericSubmapCollectionIntegrator<
    IntegratorType, IntegrationData,
    VoxelType>::getSubmapRelativePose(const Transformation& T_G_C) const {
  return (T_G_S_active_.inverse() * T_G_C);
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<
    IntegratorType, IntegrationData,
    VoxelType>::setIntegrator(std::shared_ptr<IntegratorType> integ) {
  integrator_ = integ;
}

}  // namespace cblox

#include "cblox/core/tsdf_submap.h"
#include "cblox/integrator/rgb_projection_integrator.h"
#include "cblox/integrator/thermal_projection_integrator.h"
#include "cblox/integrator/tsdf_integrator_wrapper.h"
// explicit instantiations
template class cblox::GenericSubmapCollectionIntegrator<
    cblox::TsdfIntegratorWrapper, cblox::TsdfIntegrationData,
    voxblox::TsdfVoxel>;
template class cblox::GenericSubmapCollectionIntegrator<
    cblox::RGBProjectionIntegrator<voxblox::TsdfVoxel>,
    cblox::ProjectionData<voxblox::Color>, voxblox::RGBVoxel>;
template class cblox::GenericSubmapCollectionIntegrator<
    cblox::ThermalProjectionIntegrator<voxblox::TsdfVoxel>,
    cblox::ProjectionData<float>, voxblox::IntensityVoxel>;
#include "cblox/integrator/generic_submap_collection_integrator.h"
#include "cblox/core/generic_submap.h"

#include "voxblox/integrator/tsdf_integrator.h"

namespace cblox {

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData, VoxelType>::
integrate(
    const Transformation& T_G_C, 
    const IntegrationData& data) {
    CHECK(!submap_collection_ptr_->empty())
      << "Can't integrate. No submaps in collection.";
    CHECK(integrator_)
      << "Can't integrate. Need to update integration target.";
    

    const Transformation T_S_C = getSubmapRelativePose(T_G_C);

    integrator_->integrate(T_S_C, data);
    }

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData, VoxelType>::
switchToActiveSubmap() {
    updateIntegratorTarget(submap_collection_ptr_->getActiveSubmapPtr()->getMapPtr());
    T_G_S_active_ = submap_collection_ptr_->getActiveSubmapPose();
    //TODO check if this can be done with a simple layer??, maybe reuse tsdf map?
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData, VoxelType>::
initializeIntegrator(const std::shared_ptr<GenericMap<VoxelType>>& map_ptr) {
    CHECK(map_ptr);
    //TODO, create integrator here
    void (*integration_function)(VoxelType&, IntegrationData&);
    //or maybe try to declare them explicit instead of passing?
    //TODO test


    Layer<VoxelType> *layer = new Layer<VoxelType>(0.2, 16u);
    voxblox::TsdfIntegratorBase::Config config;
    //integrator_.reset(new IntegratorType(NULL, &layer, integration_function));
    integrator_.reset(new IntegratorType("simple", config, layer));
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
void GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData, VoxelType>::
updateIntegratorTarget(const std::shared_ptr<GenericMap<VoxelType>>& map_ptr) {
    CHECK(map_ptr);

    if (integrator_ == nullptr) {
        initializeIntegrator(map_ptr);
    } else {
        std::cout << "updated integrator target" << std::endl;
        integrator_->setLayer(map_ptr->getLayerPtr());
    }
}

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
Transformation GenericSubmapCollectionIntegrator<IntegratorType, IntegrationData, VoxelType>::
getSubmapRelativePose(const Transformation& T_G_C) const {
    return (T_G_S_active_.inverse() * T_G_C);
}

} //namespace cblox

#include "cblox/integrator/tsdf_integrator_wrapper.h"
#include "cblox/core/tsdf_submap.h"
//explicit instantiations
template class cblox::GenericSubmapCollectionIntegrator<cblox::TsdfIntegratorWrapper, cblox::TsdfIntegrationData, voxblox::TsdfVoxel>; 
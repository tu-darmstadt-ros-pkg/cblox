#ifndef CBLOX_INTEGRATOR_GENERIC_SUBMAP_COLLECTION_INTEGRATOR_H_
#define CBLOX_INTEGRATOR_GENERIC_SUBMAP_COLLECTION_INTEGRATOR_H_

#include "cblox/core/common.h"
#include "cblox/core/generic_submap_collection.h"
#include "cblox/core/generic_submap.h"

//Generic Version of the tsdf_submap collection integrator
namespace cblox {

template <typename IntegratorType, typename IntegrationData, typename VoxelType>
class GenericSubmapCollectionIntegrator {
  public:
    GenericSubmapCollectionIntegrator(
        const std::shared_ptr<GenericSubmapCollection<VoxelType>>&
          submap_collection_ptr)
        : submap_collection_ptr_(submap_collection_ptr) {}
    //TODO Integrator base??, maybe change interface??

    void integrate(const Transformation& T_G_C, const IntegrationData& data);

    // Changes the active submap to the last one on the collection
    void switchToActiveSubmap();

    void setIntegrator(std::shared_ptr<IntegratorType> integ);

  protected:

    void initializeIntegrator(const std::shared_ptr<GenericMap<VoxelType>>& map_ptr);

    void updateIntegratorTarget(const std::shared_ptr<GenericMap<VoxelType>>& map_ptr);

    Transformation getSubmapRelativePose(const Transformation& T_G_C) const;

    std::shared_ptr<GenericSubmapCollection<VoxelType>> submap_collection_ptr_;

    Transformation T_G_S_active_;

    std::shared_ptr<IntegratorType> integrator_;

    void (*integration_function_)(VoxelType&, IntegrationData&);
};    

} //namespace cblox

#endif //CBLOX_INTEGRATOR_GENERIC_SUBMAP_COLLECTION_INTEGRATOR_H_
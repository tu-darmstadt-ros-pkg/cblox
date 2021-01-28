#ifndef CBLOX_INTEGRATOR_TSDF_INTEGRATOR_WRAPPER_H_
#define CBLOX_INTEGRATOR_TSDF_INTEGRATOR_WRAPPER_H_

#include <voxblox/integrator/tsdf_integrator.h>

namespace cblox {

struct TsdfIntegrationData {
  // TsdfIntegrationData()
  voxblox::Pointcloud points_C;
  voxblox::Colors colors;
  bool freespace_points;
  SubmapID id;
};

struct TsdfConfig {
  std::string type;
  voxblox::TsdfIntegratorBase::Config config;
  std::shared_ptr<GenericSubmapCollection<TsdfVoxel>> collection;

  TsdfConfig(std::string t, voxblox::TsdfIntegratorBase::Config& c,
             std::shared_ptr<GenericSubmapCollection<TsdfVoxel>> coll) {
    type = t;
    config = c;
    collection = coll;
  }
};

class TsdfIntegratorWrapper {
 public:
  TsdfIntegratorWrapper(  // const std::string& integrator_type,
      const TsdfConfig config)
      : collection_(config.collection) {
    integrator_ = voxblox::TsdfIntegratorFactory::create(
        /*integrator_type*/ config.type, config.config,
        config.collection->getActiveMapPtr()->getLayerPtr());
  }

  typedef TsdfConfig ConfigType;

  void integrate(const Transformation& T_G_C, const TsdfIntegrationData& data) {
    // TODO rethink layer updating

    setActiveLayers();
    // locking
    std::unique_lock<std::mutex> lock(collection_->collection_mutex_);
    integrator_->integratePointCloud(T_G_C, data.points_C, data.colors,
                                     data.freespace_points);
    lock.unlock();
  }

  void setActiveLayers() {
    integrator_->setLayer(collection_->getActiveMapPtr()->getLayerPtr());
  }

 protected:
  voxblox::TsdfIntegratorBase::Ptr integrator_;
  std::shared_ptr<GenericSubmapCollection<TsdfVoxel>> collection_;
};
}  // namespace cblox

#endif  // CBLOX_INTEGRATOR_TSDF_INTEGRATOR_WRAPPER_H_
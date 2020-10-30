#ifndef CBLOX_INTEGRATOR_TSDF_INTEGRATOR_WRAPPER_H_
#define CBLOX_INTEGRATOR_TSDF_INTEGRATOR_WRAPPER_H_

#include <voxblox/integrator/tsdf_integrator.h>


namespace cblox {

struct TsdfIntegrationData {
    //TsdfIntegrationData()
    voxblox::Pointcloud points_C;
    voxblox::Colors colors;
    bool freespace_points;
};

class TsdfIntegratorWrapper {
  public:
    TsdfIntegratorWrapper(//const std::string& integrator_type, 
                          const voxblox::TsdfIntegratorBase::Config& config,
                          voxblox::Layer<TsdfVoxel>* layer) {
        integrator_ = voxblox::TsdfIntegratorFactory::create(/*integrator_type*/ "simple", config, layer);
    }

    void integrate(const voxblox::Transformation& T_G_C, const TsdfIntegrationData& data) {
        integrator_->integratePointCloud(T_G_C, data.points_C, data.colors, data.freespace_points);
    }

    void setLayer(voxblox::Layer<TsdfVoxel>* layer) {
        integrator_->setLayer(layer);
    }
  protected:
    voxblox::TsdfIntegratorBase::Ptr integrator_;

};
}

#endif //CBLOX_INTEGRATOR_TSDF_INTEGRATOR_WRAPPER_H_
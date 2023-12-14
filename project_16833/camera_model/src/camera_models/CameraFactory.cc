#include "camodocal/camera_models/CameraFactory.h"

#include <boost/algorithm/string.hpp>

#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include "ceres/ceres.h"

#include <ros/ros.h>
#include <opencv2/core/persistence.hpp>

namespace camodocal
{

    boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

    CameraFactory::CameraFactory()
    {
    }

    boost::shared_ptr<CameraFactory>
    CameraFactory::instance(void)
    {
        if (m_instance.get() == 0)
        {
            m_instance.reset(new CameraFactory);
        }

        return m_instance;
    }

    CameraPtr
    CameraFactory::generateCamera(Camera::ModelType modelType,
                                  const std::string &cameraName,
                                  cv::Size imageSize) const
    {
        switch (modelType)
        {
        case Camera::KANNALA_BRANDT:
        {
            EquidistantCameraPtr camera(new EquidistantCamera);

            EquidistantCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::PINHOLE:
        {
            PinholeCameraPtr camera(new PinholeCamera);

            PinholeCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::SCARAMUZZA:
        {
            OCAMCameraPtr camera(new OCAMCamera);

            OCAMCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        case Camera::MEI:
        default:
        {
            CataCameraPtr camera(new CataCamera);

            CataCamera::Parameters params = camera->getParameters();
            params.cameraName() = cameraName;
            params.imageWidth() = imageSize.width;
            params.imageHeight() = imageSize.height;
            camera->setParameters(params);
            return camera;
        }
        }
    }

    CameraPtr
    CameraFactory::generateCameraFromYamlFile(const std::string &filename)
    {
        ROS_INFO("We are here1");
        ROS_INFO("file name: %s\n", filename.c_str());

        // cv::FileStorage fs(filename, cv::FileStorage::READ);

        cv::FileStorage fs;
        try
        {
            fs.open(filename, cv::FileStorage::READ);
        }
        catch (const cv::Exception &ex)
        {
            ROS_INFO("Failed to open the file %s\n: %s", filename.c_str(), ex.what());
            // return 0;
        }

        ROS_INFO("We are here2");
        if (!fs.isOpened())
        {
            ROS_INFO("Failed to open the file %s\n", filename.c_str());
            return CameraPtr();
        }
        else
        {
            ROS_INFO("File %s opened successfully", filename.c_str());
        }

        ROS_INFO("We are here3");
        Camera::ModelType modelType = Camera::MEI;
        if (!fs["model_type"].isNone())
        {
            std::string sModelType;
            fs["model_type"] >> sModelType;

            if (boost::iequals(sModelType, "kannala_brandt"))
            {
                modelType = Camera::KANNALA_BRANDT;
            }
            else if (boost::iequals(sModelType, "mei"))
            {
                modelType = Camera::MEI;
            }
            else if (boost::iequals(sModelType, "scaramuzza"))
            {
                modelType = Camera::SCARAMUZZA;
            }
            else if (boost::iequals(sModelType, "pinhole"))
            {
                modelType = Camera::PINHOLE;
            }
            else
            {
                std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
                return CameraPtr();
            }
        }
        ROS_INFO("We are here4");

        switch (modelType)
        {
        case Camera::KANNALA_BRANDT:
        {
            EquidistantCameraPtr camera(new EquidistantCamera);

            EquidistantCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::PINHOLE:
        {
            PinholeCameraPtr camera(new PinholeCamera);

            PinholeCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::SCARAMUZZA:
        {
            OCAMCameraPtr camera(new OCAMCamera);

            OCAMCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        case Camera::MEI:
        default:
        {
            CataCameraPtr camera(new CataCamera);

            CataCamera::Parameters params = camera->getParameters();
            params.readFromYamlFile(filename);
            camera->setParameters(params);
            return camera;
        }
        }

        ROS_INFO("We are here5");
        return CameraPtr();
    }

}

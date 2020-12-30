/*
 * Copyright (C) 2019 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef IGNITION_SENSORS_GPS_HH_
#define IGNITION_SENSORS_GPS_HH_

#include <memory>

#include <sdf/sdf.hh>

#include <ignition/common/SuppressWarning.hh>
#include <ignition/common/Time.hh>

#include <ignition/sensors/config.hh>
#include <ignition/sensors/gps/Export.hh>

#include "ignition/sensors/Sensor.hh"

namespace ignition
{
  namespace sensors
  {
    // Inline bracket to help doxygen filtering.
    inline namespace IGNITION_SENSORS_VERSION_NAMESPACE {
    //
    /// \brief forward declarations
    class GpsPrivate;

    /// \brief Gps Sensor Class
    ///
    /// An gps sensor that reports vertical position and velocity
    /// readings over ign transport
    class IGNITION_SENSORS_GPS_VISIBLE GpsSensor : public Sensor
    {
      /// \brief constructor
      public: GpsSensor();

      /// \brief destructor
      public: virtual ~GpsSensor();

      /// \brief Load the sensor based on data from an sdf::Sensor object.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(const sdf::Sensor &_sdf) override;

      /// \brief Load the sensor with SDF parameters.
      /// \param[in] _sdf SDF Sensor parameters.
      /// \return true if loading was successful
      public: virtual bool Load(sdf::ElementPtr _sdf) override;

      /// \brief Initialize values in the sensor
      /// \return True on success
      public: virtual bool Init() override;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool IGN_DEPRECATED(4)  Update(
        const ignition::common::Time &_now) override;

      /// \brief Update the sensor and generate data
      /// \param[in] _now The current time
      /// \return true if the update was successfull
      public: virtual bool Update(
        const std::chrono::steady_clock::duration &_now) override;

      /// \brief Set the latitude of the GPS
      /// \param[in] _ref Latitude of GPS in degrees
      public: void SetLatitude(double _latitude);

      /// \brief Get the latitude of the GPS
      /// \return Latitude in degrees.
      public: double Latitude() const;

      /// \brief Set the longitude of the GPS
      /// \param[in] _ref Longitude of GPS in degrees
      public: void SetLongitude(double _longitude);

      /// \brief Get the longitude of the GPS
      /// \return Longitude in degrees.
      public: double Longitude() const;

      /// \brief Set the position of the GPS
      /// \param[in] _latitude in degrees
      /// \param[in] _longitude in degrees
      public: void SetPosition(double _latitude, double _longitude);


      IGN_COMMON_WARN_IGNORE__DLL_INTERFACE_MISSING
      /// \brief Data pointer for private data
      /// \internal
      private: std::unique_ptr<GpsPrivate> dataPtr;
      IGN_COMMON_WARN_RESUME__DLL_INTERFACE_MISSING
    };
    }
  }
}

#endif

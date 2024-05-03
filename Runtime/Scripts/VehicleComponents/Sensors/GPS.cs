using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Utils = DefaultNamespace.Utils;

using DefaultNamespace.Water;

namespace VehicleComponents.Sensors
{
    public class GPS: Sensor
    {
        [Header("GPS")]
        public double easting;
        public double northing;
        public double lat;
        public double lon;
        public bool fix;

        private GPSReferencePoint _gpsRef;
        private WaterQueryModel _waterModel;

        void Start()
        {
            var gpsRefs = FindObjectsByType<GPSReferencePoint>(FindObjectsSortMode.None);
            if(gpsRefs.Length < 1)
            {
                Debug.Log("No GPS Reference found in the scene. Setting values to 0");
                easting = 0.0;
                northing = 0.0;
                lat = 0.0;
                lon = 0.0;
                fix = true;
            }
            else _gpsRef = gpsRefs[0];
            
            var waterModels = FindObjectsByType<WaterQueryModel>(FindObjectsSortMode.None);
            if(waterModels.Length < 1) Debug.Log("No water query model found. GPS will always run.");
            else _waterModel = waterModels[0];

        }

        public override void UpdateSensor(double deltaTime)
        {
            if(_gpsRef == null) return;
            if(_waterModel == null) fix = true;
            else fix = transform.position.y > _waterModel.GetWaterLevelAt(transform.position);

            // It is! We can get a fix
            if (fix) (easting, northing, lat, lon) = _gpsRef.GetUTMLatLonOfObject(gameObject);
        }
    }


}
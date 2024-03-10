using System;
using System.Linq;
using DefaultNamespace.Water;
using UnityEngine;

// This is a very simple example of how we could compute a buoyancy force at variable points along the body.
// Its not really accurate per se.
// [RequireComponent(typeof(Rigidbody))]
// [RequireComponent(typeof(IForceModel))]
namespace Force
{
    public class ForcePoint : MonoBehaviour
    {
        private Rigidbody _rigidbody;
        private int _pointCount;

        private WaterQueryModel _waterModel;

        public float depthBeforeSubmerged = 1.5f;
        public float displacementAmount = 1f;

        public GameObject motionModel;
        public bool addGravity = false;

        public Mesh volumeMesh;
        public bool automaticCenterOfGravity = false;
        private float volume;
        public float density = 997; // kg/m3
        

        public void Awake()
        {
            if (motionModel == null) Debug.Log("ForcePoints require a motionModel object with a rigidbody to function!");
            _rigidbody = motionModel.GetComponent<Rigidbody>();
            if (_rigidbody == null) _rigidbody = motionModel.transform.parent.GetComponent<Rigidbody>();
            _waterModel = FindObjectsByType<WaterQueryModel>(FindObjectsSortMode.None)[0];
            var forcePoints = transform.parent.gameObject.GetComponentsInChildren<ForcePoint>();
            if (automaticCenterOfGravity)
            {
                _rigidbody.automaticCenterOfMass = false;
                var centerOfMass = forcePoints.Select(point => point.transform.localPosition).Aggregate(new Vector3(0, 0, 0), (s, v) => s + v);
                _rigidbody.centerOfMass = centerOfMass / forcePoints.Length;
            }
            _pointCount = forcePoints.Length;
            addGravity = !_rigidbody.useGravity;
            if (volumeMesh != null) volume = MeshVolume.CalculateVolumeOfMesh(volumeMesh, Vector3.one);
        }
        // Volume * Density * Gravity
        private void FixedUpdate()
        {
            var forcePointPosition = transform.position;
            if (addGravity)
            {
                _rigidbody.AddForceAtPosition(_rigidbody.mass * Physics.gravity / _pointCount, forcePointPosition, ForceMode.Force);
            }


            float waterSurfaceLevel = _waterModel.GetWaterLevelAt(forcePointPosition);
            if (forcePointPosition.y < waterSurfaceLevel)
            {
                float displacementMultiplier = Mathf.Clamp01((waterSurfaceLevel - forcePointPosition.y) / depthBeforeSubmerged) * displacementAmount;

                _rigidbody.AddForceAtPosition(
                    volume * density * new Vector3(0, Math.Abs(Physics.gravity.y) * displacementMultiplier / _pointCount, 0),
                    forcePointPosition,
                    ForceMode.Force);
            }

        }
    }
}

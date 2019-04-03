using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using System.Text.RegularExpressions;

namespace AirSimUnity {
    /*
     * MonoBehaviour class that is attached to cameras in the scene.
     * Used for applying image filters based on settings.json or Image request by the client to record the data.
     * The three filters, Depth, Segment and Vision are supported as in Unreal.
     * Note : Most of the code is based on Unity's Image Synthesis project
     */

    [RequireComponent(typeof(Camera))]
    public class CameraFiltersScript : MonoBehaviour {
        public ImageType effect;
        public Shader effectsShader;

        private Camera myCamera;

        private static Dictionary<string, int> segmentationIds = new Dictionary<string, int>();

        private void Start() {
            myCamera = GetComponent<Camera>();
            var renderers = FindObjectsOfType<Renderer>();
            var mpb = new MaterialPropertyBlock();
            foreach (var r in renderers) {
                var id = r.gameObject.GetInstanceID();
                var layer = r.gameObject.layer;

                mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
                mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
                r.SetPropertyBlock(mpb);
                var objectName = r.gameObject.name;
                if (!segmentationIds.ContainsKey(objectName)) {
                    segmentationIds.Add(r.gameObject.name, id);
                }
            }
            UpdateCameraEffect();
        }

        public ImageType Effects {
            get {
                return effect;
            }

            set {
                effect = value;
                UpdateCameraEffect();
            }
        }

        private void OnValidate() {
            Effects = effect;
        }

        public void SetShaderEffect(ImageType type) {
            effect = type;
            UpdateCameraEffect();
        }

        public static bool SetSegmentationId(string objectName, int segmentationId, bool isNameRegex) {
            List<string> keyList = new List<string>(segmentationIds.Keys); 
            if (isNameRegex) {
                bool isValueSet = false;
                foreach (string s in keyList) {
                    if (!Regex.IsMatch(s, objectName)) {
                        continue;
                    }
                    segmentationIds[s] = segmentationId;
                    isValueSet = true;
                }
                return isValueSet;
            }

            if (!segmentationIds.ContainsKey(objectName)) {
                return false;
            }
            segmentationIds[objectName] = segmentationId;
            return true;
        }

        public static int GetSegmentationId(string objectName) {
            if (segmentationIds.ContainsKey(objectName)) {
                return segmentationIds[objectName];
            }
            return -1;
        }

        private void SetSegmentationEffect() {
            var renderers = FindObjectsOfType<Renderer>();
            var mpb = new MaterialPropertyBlock();
            foreach (var r in renderers) {
                var id = r.gameObject.GetInstanceID();
                segmentationIds.TryGetValue(r.gameObject.name, out id);
                var layer = r.gameObject.layer;

                mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
                mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
                r.SetPropertyBlock(mpb);
            }

            myCamera.renderingPath = RenderingPath.Forward;
            SetupCameraWithReplacementShader(0, Color.gray);
        }

        private void SetDepthEffect() {
            myCamera.renderingPath = RenderingPath.Forward;
            SetupCameraWithReplacementShader(2, Color.white);
        }

        private void ResetCameraEffects() {
            myCamera.renderingPath = RenderingPath.UsePlayerSettings;
            myCamera.clearFlags = CameraClearFlags.Skybox;
            myCamera.SetReplacementShader(null, null);
        }

        private void SetupCameraWithReplacementShader(int mode, Color clearColor) {
            var cb = new CommandBuffer();
            cb.SetGlobalFloat("_OutputMode", (int)mode);
            myCamera.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
            myCamera.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
            myCamera.SetReplacementShader(effectsShader, "");
            myCamera.backgroundColor = clearColor;
            myCamera.clearFlags = CameraClearFlags.SolidColor;
        }

        private void UpdateCameraEffect() {
            if (!myCamera) {
                return;
            }
            myCamera.RemoveAllCommandBuffers();
            switch (effect) {
                case ImageType.Scene:
                    ResetCameraEffects();
                    break;

                case ImageType.DepthVis:
                    SetDepthEffect();
                    break;

                case ImageType.Segmentation:
                    SetSegmentationEffect();
                    break;

                default:
                    ResetCameraEffects();
                    break;
            }
        }
    }
}

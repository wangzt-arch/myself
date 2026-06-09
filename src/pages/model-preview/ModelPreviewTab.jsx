import React, { Suspense, useEffect, useMemo, useState, useRef } from "react";
import { Canvas } from '@react-three/fiber'
import { useLoader, useThree } from '@react-three/fiber'
import * as THREE from 'three'
import {
  OrbitControls,
  Html,
  Grid,
  useProgress
} from "@react-three/drei";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";

const importAll = (r) => {
  let models = [];
  r.keys().forEach((item) => {
    models.push({ name: item.replace('./', ''), value: r(item) });
  });
  return models;
};

const modelLists = importAll(require.context('./models', false, /\.glb$/));

// 默认模型：小米SU7
const defaultModelName = 'xiaomi_su7_max.glb';
const defaultModel = modelLists.find((item) => item.name === defaultModelName)?.value || modelLists[0]?.value || "";

function ModelPreviewTab() {
  const input = useRef(null);
  const [currentModel, setCurrentModel] = useState(defaultModel);
  const [uploadedModel, setUploadedModel] = useState("");

  // 场景参数
  const [bgColor, setBgColor] = useState('#1a2332');
  const [ambientIntensity, setAmbientIntensity] = useState(0.6);
  const [ambientColor, setAmbientColor] = useState('#ffffff');
  const [pointLightIntensity, setPointLightIntensity] = useState(0.8);
  const [fillLightEnabled, setFillLightEnabled] = useState(false);
  const [fillLightIntensity, setFillLightIntensity] = useState(0.4);
  const [fillLightColor, setFillLightColor] = useState('#4488aa');
  const [fogEnabled, setFogEnabled] = useState(true);
  const [fogNear, setFogNear] = useState(8);
  const [fogFar, setFogFar] = useState(25);
  const [gridEnabled, setGridEnabled] = useState(true);
  const [gridCellColor, setGridCellColor] = useState('#2a3a50');
  const [gridSectionColor, setGridSectionColor] = useState('#3a5a7a');
  const [wireframe, setWireframe] = useState(false);
  const [autoRotate, setAutoRotate] = useState(false);
  const [autoRotateSpeed, setAutoRotateSpeed] = useState(2);
  const [modelScale, setModelScale] = useState(1.0);
  const [modelOpacity, setModelOpacity] = useState(1.0);
  const [metalness, setMetalness] = useState(0.3);
  const [roughness, setRoughness] = useState(0.5);
  const [modelColor, setModelColor] = useState('#ffffff');
  const [enableShadows, setEnableShadows] = useState(true);
  const [fov, setFov] = useState(45);
  const [showToolbar, setShowToolbar] = useState(true);

  useEffect(() => {
    return () => {
      if (uploadedModel) {
        URL.revokeObjectURL(uploadedModel);
      }
    };
  }, [uploadedModel]);

  const loadModel = () => {
    input.current.click();
  };

  const Model = () => {
    const glb = useLoader(GLTFLoader, currentModel);

    const scene = useMemo(() => {
      const cloned = glb.scene.clone(true);
      cloned.traverse((child) => {
        if (child.isMesh && child.material) {
          child.material = child.material.clone();
          if (wireframe) {
            child.material.wireframe = true;
          }
          if (modelOpacity < 1) {
            child.material.transparent = true;
            child.material.opacity = modelOpacity;
          }
          if (modelColor !== '#ffffff') {
            child.material.color = new THREE.Color(modelColor);
          }
          if (child.material.metalness !== undefined) {
            child.material.metalness = metalness;
          }
          if (child.material.roughness !== undefined) {
            child.material.roughness = roughness;
          }
        }
      });
      return cloned;
    }, [glb]);

    const fit = useMemo(() => {
      const box = new THREE.Box3().setFromObject(scene);
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const maxSize = Math.max(size.x, size.y, size.z) || 1;
      const scale = (3.2 / maxSize) * modelScale;

      return {
        position: [-center.x * scale, -center.y * scale, -center.z * scale],
        scale,
      };
    }, [scene]);

    return (
      <group position={fit.position} scale={fit.scale}>
        <primitive object={scene} />
      </group>
    );
  };

  // 相机 FOV 和阴影控制组件（放在 Canvas 内部）
  const CameraController = () => {
    const { camera, gl } = useThree();
    const prevFov = useRef(fov);
    const prevShadows = useRef(enableShadows);

    useEffect(() => {
      if (camera.fov !== fov) {
        camera.fov = fov;
        camera.updateProjectionMatrix();
      }
      prevFov.current = fov;
    }, [camera]);

    useEffect(() => {
      if (gl.shadowMap.enabled !== enableShadows) {
        gl.shadowMap.enabled = enableShadows;
      }
      prevShadows.current = enableShadows;
    }, [ gl]);

    return null;
  };

  const currentModelChange = (event) => {
    const value = event?.target.files[0];
    if (!value) return;
    if (uploadedModel) {
      URL.revokeObjectURL(uploadedModel);
    }
    const modelUrl = URL.createObjectURL(value);
    setUploadedModel(modelUrl);
    setCurrentModel(modelUrl);
  };

  const onclickToShowModel = (event) => {
    const value = event.target.value;
    setCurrentModel(value);
  };

  const selectedModel = modelLists.some((item) => item.value === currentModel)
    ? currentModel
    : "";


  return (
    <div className="model-preview__tab">
      <div className="model-preview__canvas-area">
        <Canvas camera={{ position: [0, 1.45, 5.2], fov: 45 }}>
          <CameraController />
          <color attach="background" args={[bgColor]} />
          {fogEnabled && <fog attach="fog" args={[bgColor, fogNear, fogFar]} />}
          <ambientLight intensity={ambientIntensity} color={ambientColor} />
          {fillLightEnabled && (
            <pointLight position={[-5, 5, -5]} intensity={fillLightIntensity} color={fillLightColor} />
          )}
          <Suspense fallback={<Loader />}>
            {currentModel && <Model key={currentModel} />}
            <OrbitControls
              target={[0, 0, 0]}
              enableDamping
              autoRotate={autoRotate}
              autoRotateSpeed={autoRotateSpeed}
            />
          </Suspense>
          {gridEnabled && (
            <Grid
              position={[0, -0.01, 0]}
              args={[30, 30]}
              cellSize={0.5}
              cellThickness={0.5}
              cellColor={gridCellColor}
              sectionSize={2.5}
              sectionThickness={1}
              sectionColor={gridSectionColor}
              fadeDistance={20}
              fadeStrength={1.5}
              infiniteGrid
            />
          )}
        </Canvas>
      </div>

      {/* 加载模型按钮 */}
      <div className="choose-model" onClick={loadModel}>加载模型</div>
      <input ref={input} type="file"
        className="input-model"
        onChange={currentModelChange}
        accept=".gltf,.glb" />
      <select className="select-model" value={selectedModel} onChange={onclickToShowModel}>
        <option value="" disabled>自定义模型</option>
        {
          modelLists.map((item) => {
            return <option value={item.value} label={item.name} key={item.value}></option>;
          })
        }
      </select>

      {/* 工具栏切换按钮 */}
      <div className={`toolbar-toggle ${showToolbar ? 'toolbar-toggle--active' : ''}`}
        onClick={() => setShowToolbar(!showToolbar)}
        title="场景参数"
      >
        ⚙️
      </div>

      {/* 场景参数工具条 */}
      {showToolbar && (
        <div className="scene-toolbar">
          <div className="scene-toolbar__header">
            <span>🔧 场景参数</span>
            <button className="scene-toolbar__close" onClick={() => setShowToolbar(false)}>✕</button>
          </div>

          <div className="scene-toolbar__body">
            {/* 背景颜色 */}
            <div className="toolbar-item">
              <label>背景颜色</label>
              <input type="color" value={bgColor} onChange={(e) => setBgColor(e.target.value)} />
            </div>

            {/* 相机 FOV */}
            <div className="toolbar-item">
              <label>相机FOV</label>
              <input type="range" min="20" max="120" step="1" value={fov}
                onChange={(e) => setFov(Number(e.target.value))} />
              <span className="toolbar-value">{fov}°</span>
            </div>

            <div className="toolbar-divider" />

            {/* 环境光颜色 */}
            <div className="toolbar-item">
              <label>环境光色</label>
              <input type="color" value={ambientColor} onChange={(e) => setAmbientColor(e.target.value)} />
            </div>

            {/* 环境光强度 */}
            <div className="toolbar-item">
              <label>环境光强</label>
              <input type="range" min="0" max="3" step="0.1" value={ambientIntensity}
                onChange={(e) => setAmbientIntensity(Number(e.target.value))} />
              <span className="toolbar-value">{ambientIntensity.toFixed(1)}</span>
            </div>

            {/* 主灯强度 */}
            <div className="toolbar-item">
              <label>主灯强度</label>
              <input type="range" min="0" max="5" step="0.1" value={pointLightIntensity}
                onChange={(e) => setPointLightIntensity(Number(e.target.value))} />
              <span className="toolbar-value">{pointLightIntensity.toFixed(1)}</span>
            </div>

            {/* 补光 */}
            <div className="toolbar-item toolbar-item--row">
              <label>补光</label>
              <button className={`toolbar-switch ${fillLightEnabled ? 'toolbar-switch--on' : ''}`}
                onClick={() => setFillLightEnabled(!fillLightEnabled)}>
                {fillLightEnabled ? '开' : '关'}
              </button>
            </div>

            {fillLightEnabled && (
              <>
                <div className="toolbar-item">
                  <label>补光强度</label>
                  <input type="range" min="0" max="2" step="0.1" value={fillLightIntensity}
                    onChange={(e) => setFillLightIntensity(Number(e.target.value))} />
                  <span className="toolbar-value">{fillLightIntensity.toFixed(1)}</span>
                </div>
                <div className="toolbar-item">
                  <label>补光颜色</label>
                  <input type="color" value={fillLightColor} onChange={(e) => setFillLightColor(e.target.value)} />
                </div>
              </>
            )}

            {/* 阴影 */}
            <div className="toolbar-item toolbar-item--row">
              <label>阴影</label>
              <button className={`toolbar-switch ${enableShadows ? 'toolbar-switch--on' : ''}`}
                onClick={() => setEnableShadows(!enableShadows)}>
                {enableShadows ? '开' : '关'}
              </button>
            </div>

            <div className="toolbar-divider" />

            {/* 雾效 */}
            <div className="toolbar-item toolbar-item--row">
              <label>雾效</label>
              <button className={`toolbar-switch ${fogEnabled ? 'toolbar-switch--on' : ''}`}
                onClick={() => setFogEnabled(!fogEnabled)}>
                {fogEnabled ? '开' : '关'}
              </button>
            </div>

            {fogEnabled && (
              <>
                <div className="toolbar-item">
                  <label>雾效近</label>
                  <input type="range" min="1" max="20" step="1" value={fogNear}
                    onChange={(e) => setFogNear(Number(e.target.value))} />
                  <span className="toolbar-value">{fogNear}</span>
                </div>
                <div className="toolbar-item">
                  <label>雾效远</label>
                  <input type="range" min="10" max="50" step="1" value={fogFar}
                    onChange={(e) => setFogFar(Number(e.target.value))} />
                  <span className="toolbar-value">{fogFar}</span>
                </div>
              </>
            )}

            <div className="toolbar-divider" />

            {/* 网格 */}
            <div className="toolbar-item toolbar-item--row">
              <label>网格地面</label>
              <button className={`toolbar-switch ${gridEnabled ? 'toolbar-switch--on' : ''}`}
                onClick={() => setGridEnabled(!gridEnabled)}>
                {gridEnabled ? '开' : '关'}
              </button>
            </div>

            {gridEnabled && (
              <>
                <div className="toolbar-item">
                  <label>网格细色</label>
                  <input type="color" value={gridCellColor} onChange={(e) => setGridCellColor(e.target.value)} />
                </div>
                <div className="toolbar-item">
                  <label>网格粗色</label>
                  <input type="color" value={gridSectionColor} onChange={(e) => setGridSectionColor(e.target.value)} />
                </div>
              </>
            )}

            <div className="toolbar-divider" />

            {/* 线框模式 */}
            <div className="toolbar-item toolbar-item--row">
              <label>线框模式</label>
              <button className={`toolbar-switch ${wireframe ? 'toolbar-switch--on' : ''}`}
                onClick={() => setWireframe(!wireframe)}>
                {wireframe ? '开' : '关'}
              </button>
            </div>

            {/* 自动旋转 */}
            <div className="toolbar-item toolbar-item--row">
              <label>自动旋转</label>
              <button className={`toolbar-switch ${autoRotate ? 'toolbar-switch--on' : ''}`}
                onClick={() => setAutoRotate(!autoRotate)}>
                {autoRotate ? '开' : '关'}
              </button>
            </div>

            {autoRotate && (
              <div className="toolbar-item">
                <label>旋转速度</label>
                <input type="range" min="0.5" max="10" step="0.5" value={autoRotateSpeed}
                  onChange={(e) => setAutoRotateSpeed(Number(e.target.value))} />
                <span className="toolbar-value">{autoRotateSpeed.toFixed(1)}</span>
              </div>
            )}

            <div className="toolbar-divider" />

            {/* 模型缩放 */}
            <div className="toolbar-item">
              <label>模型缩放</label>
              <input type="range" min="0.1" max="5" step="0.1" value={modelScale}
                onChange={(e) => setModelScale(Number(e.target.value))} />
              <span className="toolbar-value">{modelScale.toFixed(1)}x</span>
            </div>

            {/* 模型透明度 */}
            <div className="toolbar-item">
              <label>透明度</label>
              <input type="range" min="0.05" max="1" step="0.05" value={modelOpacity}
                onChange={(e) => setModelOpacity(Number(e.target.value))} />
              <span className="toolbar-value">{(modelOpacity * 100).toFixed(0)}%</span>
            </div>

            {/* 模型颜色 */}
            <div className="toolbar-item">
              <label>模型颜色</label>
              <input type="color" value={modelColor} onChange={(e) => setModelColor(e.target.value)} />
            </div>

            {/* 金属度 */}
            <div className="toolbar-item">
              <label>金属度</label>
              <input type="range" min="0" max="1" step="0.05" value={metalness}
                onChange={(e) => setMetalness(Number(e.target.value))} />
              <span className="toolbar-value">{metalness.toFixed(2)}</span>
            </div>

            {/* 粗糙度 */}
            <div className="toolbar-item">
              <label>粗糙度</label>
              <input type="range" min="0" max="1" step="0.05" value={roughness}
                onChange={(e) => setRoughness(Number(e.target.value))} />
              <span className="toolbar-value">{roughness.toFixed(2)}</span>
            </div>

            {/* 重置按钮 */}
            <div className="toolbar-item">
              <button className="toolbar-reset" onClick={() => {
                setBgColor('#1a2332');
                setAmbientIntensity(0.6);
                setAmbientColor('#ffffff');
                setPointLightIntensity(0.8);
                setFillLightEnabled(false);
                setFillLightIntensity(0.4);
                setFillLightColor('#4488aa');
                setFogEnabled(true);
                setFogNear(8);
                setFogFar(25);
                setGridEnabled(true);
                setGridCellColor('#2a3a50');
                setGridSectionColor('#3a5a7a');
                setWireframe(false);
                setAutoRotate(false);
                setAutoRotateSpeed(2);
                setModelScale(1.0);
                setModelOpacity(1.0);
                setModelColor('#ffffff');
                setMetalness(0.3);
                setRoughness(0.5);
                setEnableShadows(false);
                setFov(45);
              }}>
                🔄 重置默认
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

function Loader() {
  const { progress } = useProgress();
  return <Html center>{progress} % loaded</Html>;
}

export default ModelPreviewTab;

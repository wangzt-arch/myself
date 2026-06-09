import React, { Suspense, useEffect, useLayoutEffect, useMemo, useState, useRef } from "react";
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

function ModelPreviewTab() {
  const input = useRef(null);
  const defaultModel = modelLists[0]?.value || "";
  const [currentModel, setCurrentModel] = useState(defaultModel);
  const [uploadedModel, setUploadedModel] = useState("");

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
    const { camera } = useThree();

    const scene = useMemo(() => glb.scene.clone(true), [glb]);
    const fit = useMemo(() => {
      const box = new THREE.Box3().setFromObject(scene);
      const center = box.getCenter(new THREE.Vector3());
      const size = box.getSize(new THREE.Vector3());
      const maxSize = Math.max(size.x, size.y, size.z) || 1;
      const scale = 3.2 / maxSize;

      return {
        position: [-center.x * scale, -center.y * scale, -center.z * scale],
        scale,
      };
    }, [scene]);

    useLayoutEffect(() => {
      camera.position.set(0, 1.45, 5.2);
      camera.near = 0.1;
      camera.far = 1000;
      camera.lookAt(0, 0, 0);
      camera.updateProjectionMatrix();
    }, [camera]);

    return (
      <group position={fit.position} scale={fit.scale}>
        <primitive object={scene} />
      </group>
    );
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
          <color attach="background" args={['#1a2332']} />
          <fog attach="fog" args={['#1a2332', 8, 25]} />
          <ambientLight intensity={0.6} />
          <pointLight position={[10, 10, 10]} intensity={0.8} />
          <pointLight position={[-5, 5, -5]} intensity={0.3} color="#4488aa" />
          <Suspense fallback={<Loader />}>
            {currentModel && <Model key={currentModel}></Model>}
            <OrbitControls target={[0, 0, 0]} enableDamping />
          </Suspense>
          <Grid
            position={[0, -0.01, 0]}
            args={[30, 30]}
            cellSize={0.5}
            cellThickness={0.5}
            cellColor="#2a3a50"
            sectionSize={2.5}
            sectionThickness={1}
            sectionColor="#3a5a7a"
            fadeDistance={20}
            fadeStrength={1.5}
            infiniteGrid
          />
        </Canvas>
      </div>
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
    </div>
  );
}

function Loader() {
  const { progress } = useProgress();
  return <Html center>{progress} % loaded</Html>;
}

export default ModelPreviewTab;

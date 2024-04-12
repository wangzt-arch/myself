import React, { Suspense, useState } from "react";
import Header from "../../components/Header";
import { useRef } from "react";
import { Canvas } from '@react-three/fiber'
// import { FBXLoader } from 'three/examples/jsm/loaders/FBXLoader'
import { useLoader } from '@react-three/fiber'
import * as THREE from 'three'
import "./index.css";

import {
  Environment,
  OrbitControls,
  Html,
  useProgress
} from "@react-three/drei";
import { GLTFLoader } from "three/examples/jsm/loaders/GLTFLoader";
// import city from "./city.jpg"
// import blue from "./blue.jpg"
import white from "./white.jpg"





function Preview() {
  const input = useRef(null);
  const modelRef = useRef();
  let [currentModel, setCurrentModel] = useState({});
  let [flag, setFlag] = useState(false);


  const loadModel = () => {
    input.current.click()

  }
  const Model = () => {
    const glb = useLoader(GLTFLoader, currentModel);

    if (modelRef.current && glb) {
      const box = new THREE.Box3().setFromObject(glb.scene);
      const center = box.getCenter(new THREE.Vector3());
      glb.scene.position.sub(center);
    }
    return<scene><primitive object={glb.scene}  ref={modelRef}/></scene> ;
  };

  const currentModelChange = (event) => {
    console.log(event);
    const value = event.target.files[0]
    setCurrentModel(URL.createObjectURL(value))
    setFlag(false)
    setFlag(true)

  }

  return (
    <div className="model-preview">
      <Header></Header>

      <div className="model-preview__area">
        <Canvas>
        <ambientLight intensity={1} />
        <pointLight position={[10, 10, 10]} intensity={1} />
          <Suspense fallback={<Loader />}>
            {flag && <Model></Model>}
            <OrbitControls />
            <Environment  files={white} background />
          </Suspense>
        </Canvas>
      </div>
      <div className="choose-model" onClick={loadModel}>加载模型</div>
      <input ref={input} type="file" className="input-model" onChange={currentModelChange} accept=".gltf,.glb" />
    </div>
  );
}

function Loader() {
  const { progress } = useProgress();
  return <Html center>{progress} % loaded</Html>;
}

export default Preview;

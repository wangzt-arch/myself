import React, { useRef, useMemo, useEffect } from 'react';
import { useFrame, useThree } from '@react-three/fiber';
import { OrbitControls } from '@react-three/drei';
import * as THREE from 'three';
import Building from './Building';
import Ground from './Ground';

const PARK_DATA = {
  buildings: [
    { name: 'A栋 行政楼', type: '办公楼', floors: 8, position: [-4, 0, -4], size: [1.8, 2.4, 1.2], color: '#3a5a7a' },
    { name: 'B栋 研发中心', type: '研发楼', floors: 12, position: [4, 0, -4], size: [2.0, 3.6, 1.4], color: '#2a4a6a' },
    { name: 'C栋 数据中心', type: '数据中心', floors: 6, position: [-4, 0, 4], size: [2.2, 1.8, 2.0], color: '#1a3a5a' },
    { name: 'D栋 会议中心', type: '会议楼', floors: 5, position: [4, 0, 4], size: [1.6, 1.5, 1.6], color: '#4a6a8a' },
    { name: 'E栋 员工餐厅', type: '餐饮楼', floors: 3, position: [0, 0, -6], size: [2.0, 0.9, 1.2], color: '#5a7a5a' },
    { name: 'F栋 超市', type: '超市', floors: 2, position: [-6, 0, 0], size: [2.5, 1.5, 2.5], color: '#6a5a7a', externalModel: true },
    { name: 'G栋 宿舍楼A', type: '宿舍楼', floors: 10, position: [6, 0, 0], size: [1.0, 3.0, 1.0], color: '#7a6a5a' },
    { name: 'H栋 宿舍楼B', type: '宿舍楼', floors: 10, position: [7.5, 0, 0], size: [1.0, 3.0, 1.0], color: '#7a6a5a' },
    { name: 'I栋 实验楼', type: '实验楼', floors: 7, position: [0, 0, 6], size: [1.8, 2.1, 1.2], color: '#5a5a7a' },
    { name: 'J栋 图书馆', type: '图书馆', floors: 4, position: [-7, 0, -7], size: [1.4, 1.2, 1.0], color: '#7a5a5a' },
    { name: 'K栋 医疗中心', type: '医疗楼', floors: 4, position: [7, 0, -7], size: [1.2, 1.2, 1.2], color: '#5a7a7a' },
    { name: 'L栋 停车楼', type: '停车楼', floors: 5, position: [-7, 0, 7], size: [2.0, 1.5, 1.0], color: '#6a6a6a' },
  ],
};

function Scene({ onBuildingClick, selectedBuilding, viewMode, weather, timeOfDay }) {
  const controlsRef = useRef();
  const { camera } = useThree();

  // 视角切换
  useEffect(() => {
    if (!controlsRef.current) return;

    const duration = 1500;
    const startPos = camera.position.clone();
    const startTarget = controlsRef.current.target.clone();
    let endPos, endTarget;

    switch (viewMode) {
      case 'top':
        endPos = new THREE.Vector3(0, 18, 0.1);
        endTarget = new THREE.Vector3(0, 0, 0);
        break;
      case 'side':
        endPos = new THREE.Vector3(0, 3, 14);
        endTarget = new THREE.Vector3(0, 1, 0);
        break;
      case 'tour':
        endPos = new THREE.Vector3(12, 5, 12);
        endTarget = new THREE.Vector3(0, 1, 0);
        break;
      case 'overview':
      default:
        endPos = new THREE.Vector3(10, 8, 10);
        endTarget = new THREE.Vector3(0, 0, 0);
        break;
    }

    let cancelled = false;
    const startTime = Date.now();
    const animate = () => {
      if (cancelled || !controlsRef.current) return;
      const elapsed = Date.now() - startTime;
      const t = Math.min(elapsed / duration, 1);
      const ease = 1 - Math.pow(1 - t, 3);

      camera.position.lerpVectors(startPos, endPos, ease);
      controlsRef.current.target.lerpVectors(startTarget, endTarget, ease);
      controlsRef.current.update();

      if (t < 1) {
        requestAnimationFrame(animate);
      }
    };
    animate();

    return () => { cancelled = true; };
  }, [viewMode, camera]);

  // 漫游动画
  useFrame((state) => {
    if (viewMode === 'tour' && controlsRef.current) {
      const time = state.clock.elapsedTime;
      const radius = 14;
      camera.position.x = Math.sin(time * 0.15) * radius;
      camera.position.z = Math.cos(time * 0.15) * radius;
      camera.position.y = 5 + Math.sin(time * 0.1) * 2;
      camera.lookAt(0, 1, 0);
      controlsRef.current.target.set(0, 1, 0);
      controlsRef.current.update();
    }
  });

  // 光照设置
  const lightSettings = useMemo(() => {
    switch (timeOfDay) {
      case 'morning':
        return {
          ambient: 0.4,
          directional: 0.8,
          color: '#ffddaa',
          position: [10, 6, 5],
          bg: '#1a2030',
        };
      case 'evening':
        return {
          ambient: 0.3,
          directional: 0.6,
          color: '#ff8844',
          position: [-8, 4, 8],
          bg: '#1a1828',
        };
      case 'night':
        return {
          ambient: 0.15,
          directional: 0.2,
          color: '#4466aa',
          position: [5, 10, 5],
          bg: '#0a0e1a',
        };
      case 'noon':
      default:
        return {
          ambient: 0.5,
          directional: 1.0,
          color: '#ffffff',
          position: [8, 10, 5],
          bg: '#1a2332',
        };
    }
  }, [timeOfDay]);

  // 天气效果
  const weatherParticles = useMemo(() => {
    if (weather === 'rainy') {
      const particles = [];
      for (let i = 0; i < 200; i++) {
        particles.push({
          position: [
            (Math.random() - 0.5) * 20,
            Math.random() * 15 + 5,
            (Math.random() - 0.5) * 20,
          ],
          speed: 0.3 + Math.random() * 0.2,
          key: `rain-${i}`,
        });
      }
      return particles;
    }
    return [];
  }, [weather]);

  return (
    <>
      {/* 背景色 */}
      <color attach="background" args={[lightSettings.bg]} />

      {/* 雾效 */}
      <fog attach="fog" args={[lightSettings.bg, 15, 40]} />

      {/* 环境光 */}
      <ambientLight intensity={lightSettings.ambient} color={lightSettings.color} />

      {/* 方向光 */}
      <directionalLight
        position={lightSettings.position}
        intensity={lightSettings.directional}
        color={lightSettings.color}
        castShadow
        shadow-mapSize-width={1024}
        shadow-mapSize-height={1024}
      />

      {/* 点光源（夜景增强） */}
      {timeOfDay === 'night' && (
        <>
          <pointLight position={[0, 3, 0]} intensity={0.5} color="#00d4ff" distance={8} />
          <pointLight position={[-4, 2, -4]} intensity={0.3} color="#ffaa00" distance={5} />
          <pointLight position={[4, 2, 4]} intensity={0.3} color="#ffaa00" distance={5} />
        </>
      )}

      {/* 地面 */}
      <Ground
        onExternalModelClick={onBuildingClick}
        selectedBuilding={selectedBuilding}
      />

      {/* 建筑（跳过外部模型） */}
      {PARK_DATA.buildings.filter(b => !b.externalModel).map((building, index) => (
        <Building
          key={index}
          position={building.position}
          size={building.size}
          color={building.color}
          name={building.name}
          type={building.type}
          floors={building.floors}
          onClick={onBuildingClick}
          isSelected={selectedBuilding?.name === building.name}
        />
      ))}

      {/* 雨效果 */}
      {weather === 'rainy' && weatherParticles.map((p) => (
        <RainDrop key={p.key} position={p.position} speed={p.speed} />
      ))}

      {/* 控制器 */}
      <OrbitControls
        ref={controlsRef}
        target={[0, 0, 0]}
        enableDamping
        dampingFactor={0.05}
        minDistance={3}
        maxDistance={25}
        maxPolarAngle={Math.PI / 2.1}
      />
    </>
  );
}

function RainDrop({ position, speed }) {
  const ref = useRef();

  useFrame(() => {
    if (ref.current) {
      ref.current.position.y -= speed;
      if (ref.current.position.y < 0) {
        ref.current.position.y = 15;
        ref.current.position.x = (Math.random() - 0.5) * 20;
        ref.current.position.z = (Math.random() - 0.5) * 20;
      }
    }
  });

  return (
    <mesh ref={ref} position={position}>
      <cylinderGeometry args={[0.01, 0.01, 0.3, 4]} />
      <meshBasicMaterial color="#88aacc" transparent opacity={0.5} />
    </mesh>
  );
}

export { Scene, PARK_DATA };

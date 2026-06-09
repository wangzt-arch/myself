import React, { useMemo, Suspense, useRef, useState } from 'react';
import { useLoader } from '@react-three/fiber';
import { GLTFLoader } from 'three/examples/jsm/loaders/GLTFLoader';
import * as THREE from 'three';
import superMarcket from '../models/supermarket.glb';
import xiaomisu7 from '../models/xiaomi_su7_max.glb';

// 小米 SU7 外部模型组件
function XiaomiSU7({ position, rotation = 0, scale = 1 }) {
  const gltf = useLoader(GLTFLoader, xiaomisu7);
  const scene = useMemo(() => {
    const cloned = gltf.scene.clone(true);
    cloned.rotation.y = rotation;
    const box = new THREE.Box3().setFromObject(cloned);
    const size = box.getSize(new THREE.Vector3());
    const maxSize = Math.max(size.x, size.y, size.z);
    const targetScale = (1.8 / maxSize) * scale;
    cloned.scale.setScalar(targetScale);
    return cloned;
  }, [gltf, rotation, scale]);

  return (
    <primitive object={scene} position={position} />
  );
}

// 通用外部 GLB 模型组件（支持点击）
function ExternalModel({ url, position, rotation = 0, scale = 1, name, type, onClick, isSelected }) {
  const gltf = useLoader(GLTFLoader, url);
  const groupRef = useRef();
  const [hovered, setHovered] = useState(false);

  const scene = useMemo(() => {
    const cloned = gltf.scene.clone(true);
    cloned.rotation.y = rotation;
    const box = new THREE.Box3().setFromObject(cloned);
    const size = box.getSize(new THREE.Vector3());
    const maxSize = Math.max(size.x, size.y, size.z);
    const targetScale = scale / maxSize;
    cloned.scale.setScalar(targetScale);
    // 启用所有子 mesh 的射线检测
    cloned.traverse((child) => {
      if (child.isMesh) {
        child.castShadow = true;
        child.receiveShadow = true;
      }
    });
    return cloned;
  }, [gltf, rotation, scale]);

  return (
    <group
      ref={groupRef}
      position={position}
      onClick={(e) => {
        e.stopPropagation();
        if (onClick) onClick({ name, type, size: [scale, scale, scale], floors: 1 });
      }}
      onPointerOver={(e) => {
        e.stopPropagation();
        setHovered(true);
        document.body.style.cursor = 'pointer';
      }}
      onPointerOut={() => {
        setHovered(false);
        document.body.style.cursor = 'auto';
      }}
    >
      <primitive object={scene} />
      {/* 选中/悬停高亮指示器 */}
      {(isSelected || hovered) && (
        <mesh position={[0, 0.02, 0]} rotation={[-Math.PI / 2, 0, 0]}>
          <circleGeometry args={[scale * 0.6, 32]} />
          <meshBasicMaterial
            color={isSelected ? '#00d4ff' : '#00d4ff'}
            transparent
            opacity={isSelected ? 0.3 : 0.15}
          />
        </mesh>
      )}
    </group>
  );
}

function Ground({ onExternalModelClick, selectedBuilding }) {
  const gridSize = 24;
  const gridDivisions = 48;

  // 围墙路径 - 围绕整个园区
  const wallPath = useMemo(() => {
    const half = 11;
    const height = 0.8;
    const thickness = 0.15;
    const segments = [
      { pos: [0, height / 2, -half], size: [half * 2, height, thickness] },
      { pos: [0, height / 2, half], size: [half * 2, height, thickness] },
      { pos: [-half, height / 2, 0], size: [thickness, height, half * 2] },
      { pos: [half, height / 2, 0], size: [thickness, height, half * 2] },
    ];
    return segments;
  }, []);

  // 围墙柱子
  const wallPillars = useMemo(() => {
    const half = 11;
    const pillars = [];
    const spacing = 2.5;
    for (let x = -half; x <= half; x += spacing) {
      pillars.push({ pos: [x, 0.9, -half], key: `p-n-${x}` });
      pillars.push({ pos: [x, 0.9, half], key: `p-s-${x}` });
    }
    for (let z = -half + spacing; z < half; z += spacing) {
      pillars.push({ pos: [-half, 0.9, z], key: `p-w-${z}` });
      pillars.push({ pos: [half, 0.9, z], key: `p-e-${z}` });
    }
    return pillars;
  }, []);

  // 道路系统
  const roadPaths = useMemo(() => {
    const paths = [
      // 主路 - 横向贯穿
      { start: [-11, 0.01, 0], end: [11, 0.01, 0], width: 1.0 },
      // 主路 - 纵向贯穿
      { start: [0, 0.01, -11], end: [0, 0.01, 11], width: 1.0 },
      // 外环路
      { start: [-8, 0.01, -8], end: [8, 0.01, -8], width: 0.6 },
      { start: [8, 0.01, -8], end: [8, 0.01, 8], width: 0.6 },
      { start: [8, 0.01, 8], end: [-8, 0.01, 8], width: 0.6 },
      { start: [-8, 0.01, 8], end: [-8, 0.01, -8], width: 0.6 },
      // 内环路
      { start: [-4, 0.01, -4], end: [4, 0.01, -4], width: 0.5 },
      { start: [4, 0.01, -4], end: [4, 0.01, 4], width: 0.5 },
      { start: [4, 0.01, 4], end: [-4, 0.01, 4], width: 0.5 },
      { start: [-4, 0.01, 4], end: [-4, 0.01, -4], width: 0.5 },
      // 支路
      { start: [-8, 0.01, -4], end: [8, 0.01, -4], width: 0.4 },
      { start: [-8, 0.01, 4], end: [8, 0.01, 4], width: 0.4 },
      { start: [-4, 0.01, -8], end: [-4, 0.01, 8], width: 0.4 },
      { start: [4, 0.01, -8], end: [4, 0.01, 8], width: 0.4 },
    ];
    return paths;
  }, []);

  // 树木 - 更丰富
  const trees = useMemo(() => {
    const items = [];
    // 边界绿化带
    const borderPositions = [];
    for (let i = -10; i <= 10; i += 1.5) {
      borderPositions.push([i, -9.5], [i, 9.5], [-9.5, i], [9.5, i]);
    }
    // 内部绿化
    const innerPositions = [
      [-6, -6], [-6, -2], [-6, 2], [-6, 6],
      [-2, -6], [-2, 6], [2, -6], [2, 6],
      [6, -6], [6, -2], [6, 2], [6, 6],
      [-9, -9], [-9, 9], [9, -9], [9, 9],
      [-3, -9], [3, -9], [-3, 9], [3, 9],
      [-9, -3], [-9, 3], [9, -3], [9, 3],
      [-7, -3], [-7, 3], [7, -3], [7, 3],
      [-3, -7], [3, -7], [-3, 7], [3, 7],
    ];
    [...borderPositions, ...innerPositions].forEach(([x, z], i) => {
      items.push({
        position: [x + (Math.random() - 0.5) * 0.4, 0, z + (Math.random() - 0.5) * 0.4],
        scale: 0.25 + Math.random() * 0.25,
        type: Math.random() > 0.7 ? 'pine' : 'normal',
        key: `tree-${i}`,
      });
    });
    return items;
  }, []);

  // 草坪区域
  const lawns = useMemo(() => {
    return [
      { pos: [-2, 0.005, -2], size: [3, 3] },
      { pos: [2, 0.005, -2], size: [3, 3] },
      { pos: [-2, 0.005, 2], size: [3, 3] },
      { pos: [2, 0.005, 2], size: [3, 3] },
      { pos: [-6, 0.005, 0], size: [2, 4] },
      { pos: [6, 0.005, 0], size: [2, 4] },
      { pos: [0, 0.005, -6], size: [4, 2] },
      { pos: [0, 0.005, 6], size: [4, 2] },
    ];
  }, []);

  // 花坛
  const flowerBeds = useMemo(() => {
    return [
      { pos: [-1.5, 0.02, -1.5], color: '#cc4444' },
      { pos: [1.5, 0.02, -1.5], color: '#44cc44' },
      { pos: [-1.5, 0.02, 1.5], color: '#4444cc' },
      { pos: [1.5, 0.02, 1.5], color: '#cccc44' },
    ];
  }, []);

  // 停车位
  const parkingSlots = useMemo(() => {
    const slots = [];
    for (let i = 0; i < 8; i++) {
      slots.push({ pos: [8.5, 0.02, -7 + i * 1.2], rot: 0, key: `p-${i}` });
    }
    return slots;
  }, []);

  return (
    <group>
      {/* 地面 */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, -0.01, 0]} receiveShadow>
        <planeGeometry args={[gridSize, gridSize]} />
        <meshStandardMaterial
          color="#1a2332"
          metalness={0.1}
          roughness={0.8}
        />
      </mesh>

      {/* 网格线 */}
      <gridHelper
        args={[gridSize, gridDivisions, '#2a3a50', '#1e2d3d']}
        position={[0, 0, 0]}
      />

      {/* 围墙 */}
      {wallPath.map((wall, i) => (
        <mesh key={`wall-${i}`} position={wall.pos}>
          <boxGeometry args={wall.size} />
          <meshStandardMaterial
            color="#3a4a5e"
            metalness={0.3}
            roughness={0.6}
          />
        </mesh>
      ))}

      {/* 围墙顶部装饰线 */}
      {wallPath.map((wall, i) => (
        <mesh
          key={`wall-top-${i}`}
          position={[wall.pos[0], wall.pos[1] + wall.size[1] / 2 + 0.03, wall.pos[2]]}
        >
          <boxGeometry args={[wall.size[0] + 0.1, 0.06, wall.size[2] + 0.1]} />
          <meshStandardMaterial
            color="#5a7a9a"
            metalness={0.5}
            roughness={0.3}
            emissive="#5a7a9a"
            emissiveIntensity={0.2}
          />
        </mesh>
      ))}

      {/* 围墙柱子 */}
      {wallPillars.map((pillar) => (
        <mesh key={pillar.key} position={pillar.pos}>
          <boxGeometry args={[0.25, 1.0, 0.25]} />
          <meshStandardMaterial
            color="#4a5a6a"
            metalness={0.4}
            roughness={0.5}
          />
        </mesh>
      ))}

      {/* 道路 */}
      {roadPaths.map((road, i) => (
        <Road key={`road-${i}`} start={road.start} end={road.end} width={road.width} />
      ))}

      {/* 草坪 */}
      {lawns.map((lawn, i) => (
        <mesh key={`lawn-${i}`} rotation={[-Math.PI / 2, 0, 0]} position={lawn.pos}>
          <planeGeometry args={lawn.size} />
          <meshStandardMaterial
            color="#1a3a1a"
            metalness={0.1}
            roughness={0.9}
            emissive="#0a2a0a"
            emissiveIntensity={0.1}
          />
        </mesh>
      ))}

      {/* 花坛 */}
      {flowerBeds.map((bed, i) => (
        <FlowerBed key={`flower-${i}`} position={bed.pos} color={bed.color} />
      ))}

      {/* 树木 */}
      {trees.map((tree) => (
        <Tree key={tree.key} position={tree.position} scale={tree.scale} type={tree.type} />
      ))}

      {/* 停车位 */}
      {parkingSlots.map((slot) => (
        <ParkingSlot key={slot.key} position={slot.pos} />
      ))}

      {/* 车辆 - 使用小米 SU7 外部模型 */}
      <Suspense fallback={null}>
        <XiaomiSU7 position={[8.5, 0, -6]} rotation={Math.PI} />
      </Suspense>
      <Suspense fallback={null}>
        <XiaomiSU7 position={[8.5, 0, -4.2]} rotation={Math.PI} />
      </Suspense>
      <Suspense fallback={null}>
        <XiaomiSU7 position={[8.5, 0, 2.4]} rotation={0} />
      </Suspense>
      <Suspense fallback={null}>
        <XiaomiSU7 position={[8.5, 0, 3.8]} rotation={0} />
      </Suspense>
      <Suspense fallback={null}>
        <XiaomiSU7 position={[-8.5, 0, -1.2]} rotation={Math.PI} />
      </Suspense>

      {/* 路灯 */}
      <StreetLight position={[-5, 0, -5]} />
      <StreetLight position={[5, 0, -5]} />
      <StreetLight position={[-5, 0, 5]} />
      <StreetLight position={[5, 0, 5]} />
      <StreetLight position={[0, 0, -9]} />
      <StreetLight position={[0, 0, 9]} />
      <StreetLight position={[-9, 0, 0]} />
      <StreetLight position={[9, 0, 0]} />

      {/* 大门岗亭 */}
      <GateHouse position={[0, 0, 11]} />

      {/* 超市 */}
      <Suspense fallback={null}>
        <ExternalModel
          url={superMarcket}
          position={[-6, 0, 0]}
          rotation={0}
          scale={2.5}
          name="F栋 超市"
          type="超市"
          onClick={onExternalModelClick}
          isSelected={selectedBuilding?.name === 'F栋 超市'}
        />
      </Suspense>

      {/* 中心广场 */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 0]}>
        <circleGeometry args={[1.5, 32]} />
        <meshStandardMaterial
          color="#2a3a50"
          metalness={0.2}
          roughness={0.6}
        />
      </mesh>

      {/* 中心喷泉 */}
      <Fountain />
    </group>
  );
}

function Road({ start, end, width }) {
  const length = Math.sqrt(
    Math.pow(end[0] - start[0], 2) + Math.pow(end[2] - start[2], 2)
  );
  const midX = (start[0] + end[0]) / 2;
  const midZ = (start[2] + end[2]) / 2;
  const angle = Math.atan2(end[0] - start[0], end[2] - start[2]);

  return (
    <group>
      {/* 路面 */}
      <mesh
        rotation={[-Math.PI / 2, angle, 0]}
        position={[midX, 0.005, midZ]}
      >
        <planeGeometry args={[width, length]} />
        <meshStandardMaterial
          color="#3a4a5e"
          metalness={0.2}
          roughness={0.7}
        />
      </mesh>
      {/* 道路标线 */}
      <mesh
        rotation={[-Math.PI / 2, angle, 0]}
        position={[midX, 0.008, midZ]}
      >
        <planeGeometry args={[0.03, length]} />
        <meshStandardMaterial
          color="#5a7a9a"
          emissive="#5a7a9a"
          emissiveIntensity={0.3}
        />
      </mesh>
    </group>
  );
}

function Tree({ position, scale, type }) {
  const isPine = type === 'pine';
  return (
    <group position={position} scale={scale}>
      {/* 树干 */}
      <mesh position={[0, 0.3, 0]}>
        <cylinderGeometry args={[0.05, 0.07, 0.6, 6]} />
        <meshStandardMaterial color="#5a3a1a" roughness={0.9} />
      </mesh>
      {/* 树冠 */}
      {isPine ? (
        <mesh position={[0, 0.9, 0]}>
          <coneGeometry args={[0.4, 1.0, 6]} />
          <meshStandardMaterial
            color="#1a4a1a"
            roughness={0.8}
            emissive="#0a2a0a"
            emissiveIntensity={0.15}
          />
        </mesh>
      ) : (
        <>
          <mesh position={[0, 0.7, 0]}>
            <sphereGeometry args={[0.3, 8, 6]} />
            <meshStandardMaterial
              color="#2d5a27"
              roughness={0.8}
              emissive="#1a3a15"
              emissiveIntensity={0.1}
            />
          </mesh>
          <mesh position={[0.1, 0.9, 0.1]}>
            <sphereGeometry args={[0.2, 6, 4]} />
            <meshStandardMaterial
              color="#3a6a32"
              roughness={0.8}
              emissive="#1a3a15"
              emissiveIntensity={0.1}
            />
          </mesh>
        </>
      )}
    </group>
  );
}

function FlowerBed({ position, color }) {
  return (
    <group position={position}>
      {/* 花坛边框 */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.01, 0]}>
        <ringGeometry args={[0.3, 0.4, 16]} />
        <meshStandardMaterial color="#6a5a4a" roughness={0.8} />
      </mesh>
      {/* 花 */}
      <mesh position={[0, 0.08, 0]}>
        <sphereGeometry args={[0.15, 8, 6]} />
        <meshStandardMaterial
          color={color}
          emissive={color}
          emissiveIntensity={0.3}
        />
      </mesh>
    </group>
  );
}

function ParkingSlot({ position }) {
  return (
    <group position={position}>
      <mesh rotation={[-Math.PI / 2, 0, 0]}>
        <planeGeometry args={[0.8, 2.0]} />
        <meshStandardMaterial
          color="#2a3a4a"
          metalness={0.2}
          roughness={0.7}
        />
      </mesh>
      {/* 停车线 */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, -0.9]}>
        <planeGeometry args={[0.8, 0.03]} />
        <meshStandardMaterial color="#5a7a9a" emissive="#5a7a9a" emissiveIntensity={0.3} />
      </mesh>
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 0.9]}>
        <planeGeometry args={[0.8, 0.03]} />
        <meshStandardMaterial color="#5a7a9a" emissive="#5a7a9a" emissiveIntensity={0.3} />
      </mesh>
    </group>
  );
}

function GateHouse({ position }) {
  return (
    <group position={position}>
      {/* 大门柱子 - 左 */}
      <mesh position={[-1.5, 0.6, 0]}>
        <boxGeometry args={[0.4, 1.2, 0.4]} />
        <meshStandardMaterial color="#4a5a6a" metalness={0.4} roughness={0.5} />
      </mesh>
      {/* 大门柱子 - 右 */}
      <mesh position={[1.5, 0.6, 0]}>
        <boxGeometry args={[0.4, 1.2, 0.4]} />
        <meshStandardMaterial color="#4a5a6a" metalness={0.4} roughness={0.5} />
      </mesh>
      {/* 柱子顶部灯 */}
      <mesh position={[-1.5, 1.25, 0]}>
        <sphereGeometry args={[0.12, 8, 6]} />
        <meshStandardMaterial
          color="#ffcc00"
          emissive="#ffaa00"
          emissiveIntensity={1.2}
        />
      </mesh>
      <mesh position={[1.5, 1.25, 0]}>
        <sphereGeometry args={[0.12, 8, 6]} />
        <meshStandardMaterial
          color="#ffcc00"
          emissive="#ffaa00"
          emissiveIntensity={1.2}
        />
      </mesh>
      {/* 岗亭 */}
      <mesh position={[0, 0.5, 0.3]}>
        <boxGeometry args={[1.2, 1.0, 1.0]} />
        <meshStandardMaterial
          color="#3a4a5e"
          metalness={0.3}
          roughness={0.5}
          transparent
          opacity={0.85}
        />
      </mesh>
      {/* 岗亭窗户 */}
      <mesh position={[0, 0.6, 0.81]}>
        <planeGeometry args={[0.8, 0.4]} />
        <meshStandardMaterial
          color="#88ccff"
          transparent
          opacity={0.6}
          emissive="#4488cc"
          emissiveIntensity={0.4}
        />
      </mesh>
      {/* 岗亭屋顶 */}
      <mesh position={[0, 1.05, 0.3]}>
        <boxGeometry args={[1.4, 0.08, 1.2]} />
        <meshStandardMaterial color="#5a6a7a" metalness={0.5} roughness={0.3} />
      </mesh>
      {/* 道闸杆 */}
      <mesh position={[0, 0.7, -0.3]} rotation={[0, 0, Math.PI / 2]}>
        <boxGeometry args={[2.8, 0.06, 0.06]} />
        <meshStandardMaterial
          color="#ff4444"
          emissive="#ff2222"
          emissiveIntensity={0.5}
        />
      </mesh>
      {/* 道闸机 */}
      <mesh position={[-0.8, 0.25, -0.3]}>
        <boxGeometry args={[0.3, 0.5, 0.3]} />
        <meshStandardMaterial color="#4a5a6a" metalness={0.4} roughness={0.5} />
      </mesh>
      {/* 保安（简化人形） */}
      <SecurityGuard position={[0.5, 0, 0.8]} />
      {/* 园区名称牌 */}
      <mesh position={[0, 1.5, 0]}>
        <boxGeometry args={[2.5, 0.4, 0.08]} />
        <meshStandardMaterial color="#2a3a4a" metalness={0.3} roughness={0.5} />
      </mesh>
      <mesh position={[0, 1.5, 0.05]}>
        <planeGeometry args={[2.3, 0.3]} />
        <meshStandardMaterial
          color="#00d4ff"
          emissive="#00d4ff"
          emissiveIntensity={0.6}
        />
      </mesh>
    </group>
  );
}

function SecurityGuard({ position }) {
  return (
    <group position={position}>
      {/* 身体 - 制服 */}
      <mesh position={[0, 0.45, 0]}>
        <boxGeometry args={[0.28, 0.5, 0.16]} />
        <meshStandardMaterial color="#1a3a5a" roughness={0.7} />
      </mesh>
      {/* 制服反光条 */}
      <mesh position={[0, 0.55, 0.085]}>
        <boxGeometry args={[0.2, 0.04, 0.01]} />
        <meshStandardMaterial color="#ffcc00" emissive="#ffcc00" emissiveIntensity={0.5} />
      </mesh>
      <mesh position={[0, 0.4, 0.085]}>
        <boxGeometry args={[0.2, 0.04, 0.01]} />
        <meshStandardMaterial color="#ffcc00" emissive="#ffcc00" emissiveIntensity={0.5} />
      </mesh>
      {/* 肩章 */}
      <mesh position={[-0.12, 0.68, 0]}>
        <boxGeometry args={[0.06, 0.04, 0.17]} />
        <meshStandardMaterial color="#ccaa44" metalness={0.6} roughness={0.3} />
      </mesh>
      <mesh position={[0.12, 0.68, 0]}>
        <boxGeometry args={[0.06, 0.04, 0.17]} />
        <meshStandardMaterial color="#ccaa44" metalness={0.6} roughness={0.3} />
      </mesh>
      {/* 头 */}
      <mesh position={[0, 0.85, 0]}>
        <sphereGeometry args={[0.1, 8, 6]} />
        <meshStandardMaterial color="#ddaa88" roughness={0.8} />
      </mesh>
      {/* 帽子 - 大檐帽 */}
      <mesh position={[0, 0.93, 0]}>
        <cylinderGeometry args={[0.13, 0.13, 0.08, 8]} />
        <meshStandardMaterial color="#1a3a5a" roughness={0.7} />
      </mesh>
      <mesh position={[0, 0.9, 0.04]}>
        <boxGeometry args={[0.26, 0.02, 0.18]} />
        <meshStandardMaterial color="#1a3a5a" roughness={0.7} />
      </mesh>
      {/* 帽徽 */}
      <mesh position={[0, 0.95, 0.14]}>
        <circleGeometry args={[0.03, 8]} />
        <meshStandardMaterial color="#ccaa44" metalness={0.8} roughness={0.2} />
      </mesh>
      {/* 手臂 */}
      <mesh position={[-0.2, 0.5, 0]} rotation={[0, 0, 0.2]}>
        <boxGeometry args={[0.08, 0.32, 0.08]} />
        <meshStandardMaterial color="#1a3a5a" roughness={0.7} />
      </mesh>
      <mesh position={[0.2, 0.5, 0]} rotation={[0, 0, -0.2]}>
        <boxGeometry args={[0.08, 0.32, 0.08]} />
        <meshStandardMaterial color="#1a3a5a" roughness={0.7} />
      </mesh>
      {/* 手 */}
      <mesh position={[-0.22, 0.32, 0]}>
        <sphereGeometry args={[0.04, 6, 4]} />
        <meshStandardMaterial color="#ddaa88" roughness={0.8} />
      </mesh>
      <mesh position={[0.22, 0.32, 0]}>
        <sphereGeometry args={[0.04, 6, 4]} />
        <meshStandardMaterial color="#ddaa88" roughness={0.8} />
      </mesh>
      {/* 腰带 */}
      <mesh position={[0, 0.22, 0]}>
        <boxGeometry args={[0.3, 0.05, 0.18]} />
        <meshStandardMaterial color="#2a2a2a" roughness={0.8} />
      </mesh>
      {/* 腰带扣 */}
      <mesh position={[0, 0.22, 0.095]}>
        <boxGeometry args={[0.06, 0.06, 0.02]} />
        <meshStandardMaterial color="#ccaa44" metalness={0.8} roughness={0.2} />
      </mesh>
      {/* 腿 */}
      <mesh position={[-0.09, 0.12, 0]}>
        <boxGeometry args={[0.1, 0.35, 0.1]} />
        <meshStandardMaterial color="#1a2a3a" roughness={0.7} />
      </mesh>
      <mesh position={[0.09, 0.12, 0]}>
        <boxGeometry args={[0.1, 0.35, 0.1]} />
        <meshStandardMaterial color="#1a2a3a" roughness={0.7} />
      </mesh>
      {/* 鞋子 */}
      <mesh position={[-0.09, 0.03, 0.02]}>
        <boxGeometry args={[0.11, 0.06, 0.16]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.9} />
      </mesh>
      <mesh position={[0.09, 0.03, 0.02]}>
        <boxGeometry args={[0.11, 0.06, 0.16]} />
        <meshStandardMaterial color="#1a1a1a" roughness={0.9} />
      </mesh>
    </group>
  );
}

function StreetLight({ position }) {
  return (
    <group position={position}>
      {/* 灯杆 */}
      <mesh position={[0, 1.2, 0]}>
        <cylinderGeometry args={[0.04, 0.06, 2.4, 8]} />
        <meshStandardMaterial color="#4a5a6a" metalness={0.5} roughness={0.4} />
      </mesh>
      {/* 灯臂 */}
      <mesh position={[0.15, 2.3, 0]} rotation={[0, 0, -0.3]}>
        <cylinderGeometry args={[0.025, 0.03, 0.5, 6]} />
        <meshStandardMaterial color="#4a5a6a" metalness={0.5} roughness={0.4} />
      </mesh>
      {/* 灯罩 */}
      <mesh position={[0.3, 2.15, 0]}>
        <boxGeometry args={[0.2, 0.06, 0.15]} />
        <meshStandardMaterial color="#3a4a5a" metalness={0.4} roughness={0.5} />
      </mesh>
      {/* 灯光 */}
      <mesh position={[0.3, 2.1, 0]}>
        <boxGeometry args={[0.15, 0.02, 0.12]} />
        <meshStandardMaterial
          color="#ffeecc"
          emissive="#ffeecc"
          emissiveIntensity={1.5}
        />
      </mesh>
      {/* 点光源 */}
      <pointLight position={[0.3, 2.0, 0]} intensity={0.3} color="#ffeecc" distance={5} />
    </group>
  );
}

function Fountain() {
  return (
    <group position={[0, 0, 0]}>
      {/* 喷泉底座 */}
      <mesh position={[0, 0.05, 0]}>
        <cylinderGeometry args={[0.5, 0.6, 0.1, 16]} />
        <meshStandardMaterial
          color="#4a6a8a"
          metalness={0.4}
          roughness={0.3}
        />
      </mesh>
      {/* 喷泉中心柱 */}
      <mesh position={[0, 0.3, 0]}>
        <cylinderGeometry args={[0.08, 0.12, 0.4, 8]} />
        <meshStandardMaterial
          color="#5a7a9a"
          metalness={0.5}
          roughness={0.3}
        />
      </mesh>
      {/* 水柱 */}
      <mesh position={[0, 0.6, 0]}>
        <cylinderGeometry args={[0.02, 0.06, 0.3, 8]} />
        <meshStandardMaterial
          color="#00d4ff"
          transparent
          opacity={0.6}
          emissive="#00d4ff"
          emissiveIntensity={0.8}
        />
      </mesh>
      {/* 水面 */}
      <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.11, 0]}>
        <circleGeometry args={[0.5, 16]} />
        <meshStandardMaterial
          color="#00a0cc"
          transparent
          opacity={0.4}
          emissive="#00a0cc"
          emissiveIntensity={0.3}
        />
      </mesh>
    </group>
  );
}

export default Ground;

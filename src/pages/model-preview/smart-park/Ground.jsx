import React, { useMemo } from 'react';
import * as THREE from 'three';

function Ground() {
  const gridSize = 24;
  const gridDivisions = 48;

  const roadPaths = useMemo(() => {
    // 定义道路路径
    const paths = [
      // 主路 - 横向
      { start: [-10, 0.01, 0], end: [10, 0.01, 0], width: 0.8 },
      // 主路 - 纵向
      { start: [0, 0.01, -10], end: [0, 0.01, 10], width: 0.8 },
      // 环路
      { start: [-6, 0.01, -6], end: [6, 0.01, -6], width: 0.5 },
      { start: [6, 0.01, -6], end: [6, 0.01, 6], width: 0.5 },
      { start: [6, 0.01, 6], end: [-6, 0.01, 6], width: 0.5 },
      { start: [-6, 0.01, 6], end: [-6, 0.01, -6], width: 0.5 },
      // 支路
      { start: [-6, 0.01, -2], end: [6, 0.01, -2], width: 0.4 },
      { start: [-6, 0.01, 2], end: [6, 0.01, 2], width: 0.4 },
      { start: [-2, 0.01, -6], end: [-2, 0.01, 6], width: 0.4 },
      { start: [2, 0.01, -6], end: [2, 0.01, 6], width: 0.4 },
    ];
    return paths;
  }, []);

  const trees = useMemo(() => {
    const items = [];
    const positions = [
      [-5, -5], [-5, -3], [-5, 3], [-5, 5],
      [-3, -5], [-3, 5], [3, -5], [3, 5],
      [5, -5], [5, -3], [5, 3], [5, 5],
      [-7, -7], [-7, 7], [7, -7], [7, 7],
      [-4, -7], [4, -7], [-4, 7], [4, 7],
      [-7, -4], [-7, 4], [7, -4], [7, 4],
    ];
    positions.forEach(([x, z], i) => {
      items.push({
        position: [x + (Math.random() - 0.5) * 0.3, 0, z + (Math.random() - 0.5) * 0.3],
        scale: 0.3 + Math.random() * 0.2,
        key: `tree-${i}`,
      });
    });
    return items;
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

      {/* 道路 */}
      {roadPaths.map((road, i) => (
        <Road key={`road-${i}`} start={road.start} end={road.end} width={road.width} />
      ))}

      {/* 绿化带 - 树木 */}
      {trees.map((tree) => (
        <Tree key={tree.key} position={tree.position} scale={tree.scale} />
      ))}

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
  );
}

function Tree({ position, scale }) {
  return (
    <group position={position} scale={scale}>
      {/* 树干 */}
      <mesh position={[0, 0.3, 0]}>
        <cylinderGeometry args={[0.06, 0.08, 0.6, 8]} />
        <meshStandardMaterial color="#5a3a1a" roughness={0.9} />
      </mesh>
      {/* 树冠 */}
      <mesh position={[0, 0.8, 0]}>
        <sphereGeometry args={[0.35, 8, 6]} />
        <meshStandardMaterial
          color="#2d5a27"
          roughness={0.8}
          emissive="#1a3a15"
          emissiveIntensity={0.1}
        />
      </mesh>
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

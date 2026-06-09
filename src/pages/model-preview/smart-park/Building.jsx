import React, { useRef, useState, useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import * as THREE from 'three';

function Building({ position, size, color, name, type, floors, onClick, isSelected }) {
  const meshRef = useRef();
  const [hovered, setHovered] = useState(false);

  const emissiveColor = useMemo(() => {
    if (isSelected) return new THREE.Color('#00d4ff');
    if (hovered) return new THREE.Color('#00d4ff').multiplyScalar(0.3);
    return new THREE.Color('#000000');
  }, [isSelected, hovered]);

  const emissiveIntensity = useMemo(() => {
    if (isSelected) return 0.6;
    if (hovered) return 0.2;
    return 0;
  }, [isSelected, hovered]);

  useFrame((state) => {
    if (meshRef.current) {
      meshRef.current.material.emissive = emissiveColor;
      meshRef.current.material.emissiveIntensity = emissiveIntensity;
    }
  });

  return (
    <group position={position}>
      {/* 建筑主体 */}
      <mesh
        ref={meshRef}
        position={[0, size[1] / 2, 0]}
        onClick={(e) => {
          e.stopPropagation();
          onClick({ position, size, color, name, type, floors });
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
        <boxGeometry args={size} />
        <meshStandardMaterial
          color={color}
          metalness={0.3}
          roughness={0.4}
          transparent
          opacity={0.92}
        />
      </mesh>

      {/* 建筑顶部发光条 */}
      <mesh position={[0, size[1] + 0.02, 0]}>
        <boxGeometry args={[size[0] * 0.9, 0.04, size[2] * 0.9]} />
        <meshStandardMaterial
          color={isSelected ? '#00d4ff' : color}
          emissive={isSelected ? '#00d4ff' : color}
          emissiveIntensity={isSelected ? 1.2 : 0.4}
        />
      </mesh>

      {/* 窗户网格效果 */}
      <Windows width={size[0]} height={size[1]} depth={size[2]} />

      {/* 建筑名称标签 */}
      {isSelected && (
        <mesh position={[0, size[1] + 0.5, 0]}>
          <planeGeometry args={[1.2, 0.3]} />
          <meshBasicMaterial color="#0a1628" transparent opacity={0.85} />
        </mesh>
      )}
    </group>
  );
}

function Windows({ width, height, depth }) {
  const windowRows = Math.floor(height * 3);
  const windowColsFront = Math.floor(width * 2);
  const windowColsSide = Math.floor(depth * 2);

  const windows = useMemo(() => {
    const items = [];
    const spacingY = height / (windowRows + 1);
    const winSize = 0.08;

    // 前面窗户
    const spacingXFront = width / (windowColsFront + 1);
    for (let r = 1; r <= windowRows; r++) {
      for (let c = 1; c <= windowColsFront; c++) {
        const lit = Math.random() > 0.3;
        items.push({
          pos: [
            -width / 2 + spacingXFront * c,
            spacingY * r,
            depth / 2 + 0.01
          ],
          size: [winSize, winSize, 0.01],
          lit,
          key: `f-${r}-${c}`,
        });
      }
    }

    // 后面窗户
    for (let r = 1; r <= windowRows; r++) {
      for (let c = 1; c <= windowColsFront; c++) {
        const lit = Math.random() > 0.3;
        items.push({
          pos: [
            -width / 2 + spacingXFront * c,
            spacingY * r,
            -depth / 2 - 0.01
          ],
          size: [winSize, winSize, 0.01],
          lit,
          key: `b-${r}-${c}`,
        });
      }
    }

    // 侧面窗户
    const spacingXSide = depth / (windowColsSide + 1);
    for (let r = 1; r <= windowRows; r++) {
      for (let c = 1; c <= windowColsSide; c++) {
        const lit = Math.random() > 0.3;
        items.push({
          pos: [
            width / 2 + 0.01,
            spacingY * r,
            -depth / 2 + spacingXSide * c
          ],
          size: [0.01, winSize, winSize],
          lit,
          key: `r-${r}-${c}`,
        });
      }
    }

    for (let r = 1; r <= windowRows; r++) {
      for (let c = 1; c <= windowColsSide; c++) {
        const lit = Math.random() > 0.3;
        items.push({
          pos: [
            -width / 2 - 0.01,
            spacingY * r,
            -depth / 2 + spacingXSide * c
          ],
          size: [0.01, winSize, winSize],
          lit,
          key: `l-${r}-${c}`,
        });
      }
    }

    return items;
  }, [width, height, depth, windowRows, windowColsFront, windowColsSide]);

  return (
    <group>
      {windows.map((w) => (
        <mesh key={w.key} position={w.pos}>
          <boxGeometry args={w.size} />
          <meshStandardMaterial
            color={w.lit ? '#ffeb3b' : '#1a2332'}
            emissive={w.lit ? '#ffcc00' : '#000000'}
            emissiveIntensity={w.lit ? 0.5 : 0}
          />
        </mesh>
      ))}
    </group>
  );
}

export default Building;

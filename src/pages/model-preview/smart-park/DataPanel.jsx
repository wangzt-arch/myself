import React from 'react';

function DataPanel({ selectedBuilding, parkData, onBuildingSelect }) {
  const buildings = parkData?.buildings || [];
  const totalBuildings = buildings.length;
  const totalFloors = buildings.reduce((sum, b) => sum + (b.floors || 0), 0);
  const onlineDevices = Math.floor(Math.random() * 20 + 80);
  const totalEnergy = Math.floor(Math.random() * 500 + 2000);

  return (
    <div className="smart-park__data-panel">
      {/* 园区概览 */}
      <div className="data-panel__section">
        <div className="data-panel__title">📊 园区概览</div>
        <div className="data-panel__stats">
          <div className="stat-item">
            <div className="stat-value">{totalBuildings}</div>
            <div className="stat-label">建筑数量</div>
          </div>
          <div className="stat-item">
            <div className="stat-value">{totalFloors}</div>
            <div className="stat-label">总楼层</div>
          </div>
          <div className="stat-item">
            <div className="stat-value">{onlineDevices}%</div>
            <div className="stat-label">设备在线</div>
          </div>
          <div className="stat-item">
            <div className="stat-value">{totalEnergy}</div>
            <div className="stat-label">总能耗(kW)</div>
          </div>
        </div>
      </div>

      {/* 建筑列表 */}
      <div className="data-panel__section">
        <div className="data-panel__title">🏢 建筑列表</div>
        <div className="building-list">
          {buildings.map((building, index) => (
            <div
              key={index}
              className={`building-item ${selectedBuilding?.name === building.name ? 'building-item--active' : ''}`}
              onClick={() => onBuildingSelect(building)}
            >
              <div className="building-item__name">{building.name}</div>
              <div className="building-item__info">
                <span>{building.type}</span>
                <span>{building.floors}层</span>
              </div>
            </div>
          ))}
        </div>
      </div>

    </div>
  );
}

export default DataPanel;

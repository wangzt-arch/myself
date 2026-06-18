import { useState } from 'react';
import './colorPicker.css';

export default function ColorPicker({ initialColor = '#00c8ff', onConfirm, onCancel }) {
  const [color, setColor] = useState(initialColor);

  const handleConfirm = () => {
    if (onConfirm) {
      onConfirm(color);
    }
  };

  return (
    <div className="color-picker-overlay" onClick={onCancel}>
      <div className="color-picker-modal" onClick={(e) => e.stopPropagation()}>
        <div className="color-picker-buttons">
          <button className="color-picker-btn cancel" onClick={onCancel}>取消</button>
          <button className="color-picker-btn confirm" onClick={handleConfirm}>确定</button>
        </div>
        <input
          type="color"
          className="color-picker-trigger"
          value={color}
          onChange={(e) => setColor(e.target.value)}
        />
      </div>
    </div>
  );
}
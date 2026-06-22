import { useState, useMemo, useEffect, useCallback } from 'react';
import './index.css';

const PRESETS = [
  { name: '柔和阴影', offsetX: 0, offsetY: 8, blur: 20, spread: 0, color: '#1e3a5f', opacity: 0.4 },
  { name: '硬阴影', offsetX: 4, offsetY: 4, blur: 0, spread: 2, color: '#0f172a', opacity: 0.8 },
  { name: '发光效果', offsetX: 0, offsetY: 0, blur: 25, spread: 5, color: '#ef4444', opacity: 0.6 },
  { name: '悬浮效果', offsetX: 0, offsetY: 12, blur: 16, spread: -2, color: '#1e3a5f', opacity: 0.5 },
  { name: '底部阴影', offsetX: 0, offsetY: 20, blur: 40, spread: 0, color: '#0f172a', opacity: 0.6 },
  { name: '霓虹发光', offsetX: 0, offsetY: 0, blur: 30, spread: 0, color: '#06b6d4', opacity: 0.8 },
];

const STORAGE_KEY = 'css-generator-config';

function Toast({ message, visible, onClose }) {
  useEffect(() => {
    if (visible) {
      const timer = setTimeout(onClose, 2000);
      return () => clearTimeout(timer);
    }
  }, [visible, onClose]);

  return (
    <div className={`css-gen__toast ${visible ? 'css-gen__toast--visible' : ''}`}>
      <span className="css-gen__toast-icon">✓</span>
      <span className="css-gen__toast-text">{message}</span>
    </div>
  );
}

function ShadowInput({ label, value, onChange, min, max, step = 1 }) {
  const [isEditing, setIsEditing] = useState(false);
  const [inputValue, setInputValue] = useState(value.toString());

  useEffect(() => {
    if (!isEditing) {
      setInputValue(value.toString());
    }
  }, [value, isEditing]);

  const handleInputChange = (e) => {
    setInputValue(e.target.value);
  };

  const handleInputBlur = () => {
    setIsEditing(false);
    const parsed = parseFloat(inputValue);
    if (!isNaN(parsed)) {
      onChange(Math.max(min, Math.min(max, parsed)));
    } else {
      setInputValue(value.toString());
    }
  };

  const handleInputKeyDown = (e) => {
    if (e.key === 'Enter') {
      handleInputBlur();
    } else if (e.key === 'Escape') {
      setIsEditing(false);
      setInputValue(value.toString());
    } else if (e.key === 'ArrowUp') {
      e.preventDefault();
      const newValue = Math.min(max, value + (step || 1));
      onChange(newValue);
      setInputValue(newValue.toString());
    } else if (e.key === 'ArrowDown') {
      e.preventDefault();
      const newValue = Math.max(min, value - (step || 1));
      onChange(newValue);
      setInputValue(newValue.toString());
    }
  };

  const handleSliderChange = (e) => {
    const newValue = Number(e.target.value);
    onChange(newValue);
    setInputValue(newValue.toString());
  };

  const percentage = ((value - min) / (max - min)) * 100;

  return (
    <div className="css-gen__row">
      <label className="css-gen__label">
        <span>{label}</span>
        <div className="css-gen__val-wrap">
          <input
            type="number"
            className={`css-gen__val-input ${isEditing ? 'css-gen__val-input--editing' : ''}`}
            value={inputValue}
            onChange={handleInputChange}
            onFocus={() => setIsEditing(true)}
            onBlur={handleInputBlur}
            onKeyDown={handleInputKeyDown}
            min={min}
            max={max}
            step={step}
          />
          <span className="css-gen__val-unit">px</span>
        </div>
      </label>
      <div className="css-gen__slider-wrap">
        <input
          type="range"
          min={min}
          max={max}
          value={value}
          onChange={handleSliderChange}
          className="css-gen__slider"
        />
        <div className="css-gen__slider-track" style={{ width: `${percentage}%` }} />
        <div className="css-gen__slider-thumb" style={{ left: `${percentage}%` }} />
      </div>
    </div>
  );
}

function ShadowItem({ shadow, onChange }) {
  return (
    <div className="css-gen__shadow-item">
      <ShadowInput
        label="X 偏移"
        value={shadow.offsetX}
        onChange={(v) => onChange(0, { ...shadow, offsetX: v })}
        onBlur={(v) => onChange(0, { ...shadow, offsetX: v })}
        min="-100"
        max="100"
      />
      <ShadowInput
        label="Y 偏移"
        value={shadow.offsetY}
        onChange={(v) => onChange(0, { ...shadow, offsetY: v })}
        onBlur={(v) => onChange(0, { ...shadow, offsetY: v })}
        min="-100"
        max="100"
      />
      <ShadowInput
        label="模糊半径"
        value={shadow.blur}
        onChange={(v) => onChange(0, { ...shadow, blur: v })}
        onBlur={(v) => onChange(0, { ...shadow, blur: v })}
        min="0"
        max="200"
      />
      <ShadowInput
        label="扩散半径"
        value={shadow.spread}
        onChange={(v) => onChange(0, { ...shadow, spread: v })}
        onBlur={(v) => onChange(0, { ...shadow, spread: v })}
        min="-50"
        max="100"
      />
      <div className="css-gen__row css-gen__row--color">
        <label className="css-gen__label">
          <span>阴影颜色</span>
        </label>
        <div className="css-gen__color-wrap">
          <input
            type="color"
            value={shadow.color}
            onChange={(e) => onChange(0, { ...shadow, color: e.target.value })}
            onBlur={(e) => onChange(0, { ...shadow, color: e.target.value })}
            className="css-gen__color"
          />
          <span className="css-gen__color-hex">{shadow.color.toUpperCase()}</span>
        </div>
      </div>
      <div className="css-gen__row">
        <label className="css-gen__label">
          <span>不透明度</span>
          <span className="css-gen__val">{(shadow.opacity * 100).toFixed(0)}%</span>
        </label>
        <div className="css-gen__slider-wrap">
          <input
            type="range"
            min="0"
            max="100"
            value={shadow.opacity * 100}
            onChange={(e) => onChange(0, { ...shadow, opacity: Number(e.target.value) / 100 })}
            onBlur={(e) => onChange(0, { ...shadow, opacity: Number(e.target.value) / 100 })}
            className="css-gen__slider"
          />
          <div className="css-gen__slider-track" style={{ width: `${shadow.opacity * 100}%` }} />
          <div className="css-gen__slider-thumb" style={{ left: `${shadow.opacity * 100}%` }} />
        </div>
      </div>
      <div className="css-gen__row css-gen__row--toggle">
        <label className="css-gen__label">
          <span>内阴影</span>
        </label>
        <label className="css-gen__toggle">
          <input
            type="checkbox"
            checked={shadow.inset}
            onChange={(e) => onChange(0, { ...shadow, inset: e.target.checked })}
            onBlur={(e) => onChange(0, { ...shadow, inset: e.target.checked })}
          />
          <span className="css-gen__toggle-track" />
          <span className="css-gen__toggle-thumb" />
        </label>
      </div>
    </div>
  );
}

export default function CssGenerator() {
  const [shadows, setShadows] = useState([
    {
      offsetX: 0,
      offsetY: 10,
      blur: 20,
      spread: 0,
      color: '#ef4444',
      opacity: 0.5,
      inset: false,
    },
  ]);
  const [blurFilter, setBlurFilter] = useState(0);
  const [previewColor, setPreviewColor] = useState('#ffffff');
  const [copied, setCopied] = useState(false);
  const [toastVisible, setToastVisible] = useState(false);
  const [toastMessage, setToastMessage] = useState('');
  const [codeFormat, setCodeFormat] = useState('css');

  useEffect(() => {
    const saved = localStorage.getItem(STORAGE_KEY);
    if (saved) {
      try {
        const parsed = JSON.parse(saved);
        if (parsed.shadows) setShadows(parsed.shadows);
        if (parsed.blurFilter !== undefined) setBlurFilter(parsed.blurFilter);
        if (parsed.previewColor) setPreviewColor(parsed.previewColor);
      } catch (e) {
        console.error('Failed to load saved config', e);
      }
    }
  }, []);

  useEffect(() => {
    localStorage.setItem(STORAGE_KEY, JSON.stringify({ shadows, blurFilter, previewColor }));
  }, [shadows, blurFilter, previewColor]);

  const showToast = useCallback((message) => {
    setToastMessage(message);
    setToastVisible(true);
  }, []);

  const rgbaColor = (color, opacity) => {
    const safeColor = color || '#ef4444';
    const hex = safeColor.replace('#', '');
    const r = parseInt(hex.substring(0, 2), 16);
    const g = parseInt(hex.substring(2, 4), 16);
    const b = parseInt(hex.substring(4, 6), 16);
    return `rgba(${r}, ${g}, ${b}, ${opacity})`;
  };

  const boxShadow = useMemo(() => {
    return shadows
      .map(
        (s) =>
          `${s.inset ? 'inset ' : ''}${s.offsetX}px ${s.offsetY}px ${s.blur}px ${s.spread}px ${rgbaColor(s.color, s.opacity)}`
      )
      .join(', ');
  }, [shadows]);

  const cssCode = useMemo(() => {
    const base = `box-shadow: ${boxShadow};`;
    switch (codeFormat) {
      case 'css':
        return `.element {\n  ${base}\n}`;
      case 'scss':
        return `$shadow-offset-x: ${shadows[0]?.offsetX || 0}px;\n$shadow-offset-y: ${shadows[0]?.offsetY || 0}px;\n$shadow-blur: ${shadows[0]?.blur || 0}px;\n$shadow-spread: ${shadows[0]?.spread || 0}px;\n$shadow-color: ${rgbaColor(shadows[0]?.color || '#ef4444', shadows[0]?.opacity || 0.5)};\n\n@mixin shadow {\n  box-shadow: $shadow-offset-x $shadow-offset-y $shadow-blur $shadow-spread $shadow-color;\n}\n\n.element {\n  @include shadow;\n}`;
      case 'tailwind':
        const s = shadows[0];
        if (!s) return '';
        const parts = [];
        if (s.inset) parts.push('inset');
        if (s.offsetX !== 0) parts.push(s.offsetX > 0 ? `shadow-[${s.offsetX}px_` : `shadow-[-${Math.abs(s.offsetX)}px_`);
        if (s.offsetY !== 0) parts.push(s.offsetY > 0 ? `${s.offsetY}px_` : `-${Math.abs(s.offsetY)}px_`);
        parts.push(`${s.blur}px_${s.spread}px_${rgbaColor(s.color, s.opacity)}]`);
        return `className="${parts.join('')}"`;
      default:
        return base;
    }
  }, [boxShadow, shadows, codeFormat]);

  const adjustColorBrightness = (color, percent) => {
    const num = parseInt(color.replace('#', ''), 16);
    const amt = Math.round(2.55 * percent);
    const R = (num >> 16) + amt;
    const G = (num >> 8 & 0x00FF) + amt;
    const B = (num & 0x0000FF) + amt;
    return '#' + (
      0x1000000 +
      (R < 255 ? R < 1 ? 0 : R : 255) * 0x10000 +
      (G < 255 ? G < 1 ? 0 : G : 255) * 0x100 +
      (B < 255 ? B < 1 ? 0 : B : 255)
    ).toString(16).slice(1);
  };

  const previewStyle = useMemo(() => ({
    boxShadow,
    background: ` ${previewColor}`,
  }), [boxShadow, previewColor]);

  const copy = () => {
    navigator.clipboard.writeText(cssCode);
    setCopied(true);
    showToast('代码已复制到剪贴板');
    setTimeout(() => setCopied(false), 2000);
  };

  const reset = () => {
    const currentColor = shadows[0]?.color || '#ef4444';
    const currentOpacity = shadows[0]?.opacity || 0.5;
    setShadows([
      {
        offsetX: 0,
        offsetY: 10,
        blur: 20,
        spread: 0,
        color: currentColor,
        opacity: currentOpacity,
        inset: false,
      },
    ]);
    setBlurFilter(0);
    showToast('已重置为默认值');
  };

  const applyPreset = (preset) => {
    const currentColor = shadows[0]?.color || '#ef4444';
    const currentOpacity = shadows[0]?.opacity || 0.5;
    setShadows([
      {
        ...preset,
        color: currentColor,
        opacity: currentOpacity,
        inset: false,
      },
    ]);
    showToast(`已应用「${preset.name}」预设`);
  };

  const updateShadow = (index, updatedShadow) => {
    const newShadows = [...shadows];
    newShadows[index] = updatedShadow;
    setShadows(newShadows);
  };

  return (
    <div className="css-gen">
      <Toast message={toastMessage} visible={toastVisible} onClose={() => setToastVisible(false)} />

      <div className="css-gen__main">
        <div className="css-gen__preview-area">
          <div className="css-gen__preview-color-picker">
            <span className="css-gen__preview-color-label">预览方块颜色</span>
            <div className="css-gen__preview-color-wrap">
              <input
                type="color"
                value={previewColor}
                onChange={(e) => setPreviewColor(e.target.value)}
                className="css-gen__preview-color-input"
              />
              <span className="css-gen__preview-color-hex">{previewColor.toUpperCase()}</span>
            </div>
          </div>
          <div className="css-gen__preview-wrap">
            <div className="css-gen__preview-box" style={previewStyle} />
            <div className="css-gen__preview-hint">预览效果</div>
          </div>
        </div>

        <div className="css-gen__controls">
          <div className="css-gen__section">
            <div className="css-gen__section-title">阴影配置</div>
            <div className="css-gen__presets">
              {PRESETS.map((preset, idx) => (
                <button
                  key={idx}
                  className="css-gen__preset-btn"
                  onClick={() => applyPreset(preset)}
                >
                  {preset.name}
                </button>
              ))}
            </div>
            <div className="css-gen__shadows-list">
              <ShadowItem
                key={0}
                shadow={shadows[0]}
                onChange={updateShadow}
              />
            </div>
          </div>
        </div>
      </div>

      <div className="css-gen__code-area">
        <div className="css-gen__code-header">
          <div className="css-gen__code-header-left">
            <span className="css-gen__code-title">生成的代码</span>
            <div className="css-gen__code-formats">
              {[
                { value: 'css', label: 'CSS' },
                { value: 'scss', label: 'SCSS' },
                { value: 'tailwind', label: 'Tailwind' },
              ].map((format) => (
                <button
                  key={format.value}
                  className={`css-gen__format-btn ${codeFormat === format.value ? 'active' : ''}`}
                  onClick={() => setCodeFormat(format.value)}
                >
                  {format.label}
                </button>
              ))}
            </div>
          </div>
          <div className="css-gen__code-actions">
            <button className="css-gen__btn css-gen__btn--ghost" onClick={reset}>
              重置
            </button>
            <button className={`css-gen__btn css-gen__btn--primary ${copied ? 'copied' : ''}`} onClick={copy}>
              {copied ? '✓ 已复制' : '复制代码'}
            </button>
          </div>
        </div>
        <div className="css-gen__code-wrap">
          <pre className="css-gen__code">
            <code>{cssCode}</code>
          </pre>
        </div>
      </div>
    </div>
  );
}
import React, { useMemo, useState } from "react";
import { EFFECT_TYPES } from "../effects";

const EFFECT_FILTERS = [
  { key: "all", label: "全部" },
  { key: "fire", label: "火焰" },
  { key: "blast", label: "爆炸" },
  { key: "smoke", label: "烟雾" },
  { key: "assist", label: "辅助" },
];

const EFFECT_GROUPS = [
  { key: "fire", label: "火焰", effects: ["flame", "fireball"] },
  { key: "blast", label: "爆炸", effects: ["explosion", "circleBlast"] },
  { key: "smoke", label: "烟雾", effects: ["volumeSmoke"] },
  { key: "assist", label: "辅助", effects: ["circleWarn", "energyWall", "alarmWall", "glow"] },
];

const EFFECT_GROUP_LABELS = EFFECT_GROUPS.reduce((labels, group) => {
  group.effects.forEach((effectKey) => {
    labels[effectKey] = group.label;
  });
  return labels;
}, {});

function EffectLibraryPanel({
  isOpen,
  activeEffectType,
  webGLEffects,
  onToggle,
  onSelectEffect,
  onCancel,
  onAddAtViewCenter,
  onClearEffects,
  onDeleteEffect,
}) {
  const [activeEffectFilter, setActiveEffectFilter] = useState("all");
  const activeEffect = useMemo(
    () => EFFECT_TYPES.find((effect) => effect.key === activeEffectType) || null,
    [activeEffectType]
  );
  const visibleEffectGroups = useMemo(() => {
    return EFFECT_GROUPS
      .filter((group) => activeEffectFilter === "all" || group.key === activeEffectFilter)
      .map((group) => ({
        ...group,
        items: group.effects
          .map((effectKey) => EFFECT_TYPES.find((effect) => effect.key === effectKey))
          .filter(Boolean),
      }))
      .filter((group) => group.items.length > 0);
  }, [activeEffectFilter]);

  return (
    <div className={isOpen ? "effect-panel" : "effect-panel effect-panel--collapsed"}>
      <div className="effect-panel__header">
        <div>
          <span>Effect Library</span>
          <strong>特效库</strong>
        </div>
        <button type="button" onClick={onToggle}>
          {isOpen ? "收起" : "展开"}
        </button>
      </div>

      {isOpen && (
        <div className="effect-panel__body">
          <div className="effects-panel__title">
            <div>
              <span>WebGL Effects</span>
              <strong>选择特效</strong>
            </div>
            <div>
              <button type="button" onClick={onCancel} disabled={!activeEffectType}>
                取消
              </button>
              <button type="button" onClick={onAddAtViewCenter} disabled={!activeEffectType}>
                在中心添加
              </button>
              <button type="button" onClick={onClearEffects} disabled={!webGLEffects.length}>
                清空
              </button>
            </div>
          </div>

          <div className="effect-current">
            <span>当前</span>
            <strong>{activeEffect ? activeEffect.label : "未选择"}</strong>
            {activeEffect && <i style={{ backgroundColor: activeEffect.color }}></i>}
          </div>

          <div className="effect-tabs" role="tablist" aria-label="effect categories">
            {EFFECT_FILTERS.map((filter) => (
              <button
                className={activeEffectFilter === filter.key ? "effect-tab effect-tab--active" : "effect-tab"}
                key={filter.key}
                type="button"
                onClick={() => setActiveEffectFilter(filter.key)}
              >
                {filter.label}
              </button>
            ))}
          </div>

          <div className="effect-library">
            {visibleEffectGroups.map((group) => (
              <div className="effect-group" key={group.key}>
                <div className="effect-group__title">{group.label}</div>
                <div className="effect-cards">
                  {group.items.map((effect) => (
                    <button
                      className={activeEffectType === effect.key ? "effect-card effect-card--active" : "effect-card"}
                      key={effect.key}
                      type="button"
                      style={{ "--effect-color": effect.color }}
                      onClick={() => onSelectEffect(effect.key)}
                    >
                      <i></i>
                      <span>{effect.label}</span>
                      <em>{EFFECT_GROUP_LABELS[effect.key]}</em>
                    </button>
                  ))}
                </div>
              </div>
            ))}
          </div>

          <p className="effects-panel__hint">
            {activeEffectType ? "点击地图添加当前特效" : "选择一种特效后点击地图添加"}
          </p>

          <div className="effect-list__title">场景中的特效</div>
          <div className="effect-list">
            {webGLEffects.length ? (
              webGLEffects.map((effect) => (
                <div className="effect-item" key={effect.id}>
                  <div>
                    <strong>{effect.label}</strong>
                    <span>{effect.lon}, {effect.lat}</span>
                  </div>
                  <button type="button" onClick={() => onDeleteEffect(effect.id)}>删除</button>
                </div>
              ))
            ) : (
              <div className="effect-empty">暂无特效</div>
            )}
          </div>
        </div>
      )}
    </div>
  );
}

export default EffectLibraryPanel;

import { useEffect, useRef, useState } from "react";

/**
 * useScrollReveal - 监听元素进入视口时触发入场动画
 * @param {Object} options
 * @param {number} options.threshold - 元素露出比例触发阈值，默认 0.15
 * @param {string} options.rootMargin - 视口边距，默认 "0px 0px -40px 0px"
 * @returns {{ ref: React.RefObject, isVisible: boolean }}
 */
export default function useScrollReveal(options = {}) {
  const { threshold = 0.15, rootMargin = "0px 0px -40px 0px" } = options;
  const ref = useRef(null);
  const [isVisible, setIsVisible] = useState(false);

  useEffect(() => {
    const node = ref.current;
    if (!node) return;

    // 如果元素首次加载时已在视口内，直接显示（避免首屏白屏）
    const rect = node.getBoundingClientRect();
    if (rect.top < window.innerHeight && rect.bottom > 0) {
      setIsVisible(true);
    }

    const observer = new IntersectionObserver(
      ([entry]) => {
        if (entry.isIntersecting) {
          setIsVisible(true);
          observer.unobserve(node); // 只触发一次
        }
      },
      { threshold, rootMargin }
    );

    observer.observe(node);
    return () => observer.disconnect();
  }, [threshold, rootMargin]);

  return { ref, isVisible };
}

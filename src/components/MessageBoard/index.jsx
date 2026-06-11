import React, { useState, useEffect, useCallback, useRef } from "react";
import { sendMessage } from "../../api";
import "./index.scss";

const MAX_DAILY_MESSAGES = 5;
const STORAGE_KEY = "message_board_daily_count";
const THROTTLE_TIME = 2000; // 2秒节流

function generateCaptcha() {
    const a = Math.floor(Math.random() * 10) + 1;
    const b = Math.floor(Math.random() * 10) + 1;
    const operators = ["+", "-", "*"];
    const op = operators[Math.floor(Math.random() * operators.length)];
    let answer;
    switch (op) {
        case "+": answer = a + b; break;
        case "-": answer = a - b; break;
        case "*": answer = a * b; break;
        default: answer = a + b;
    }
    return { question: `${a} ${op} ${b} = ?`, answer };
}

function getTodayKey() {
    const today = new Date();
    return `${STORAGE_KEY}_${today.getFullYear()}_${today.getMonth() + 1}_${today.getDate()}`;
}

function getDailyCount() {
    const key = getTodayKey();
    const count = localStorage.getItem(key);
    return count ? parseInt(count, 10) : 0;
}

function incrementDailyCount() {
    const key = getTodayKey();
    const currentCount = getDailyCount();
    localStorage.setItem(key, currentCount + 1);
}

function MessageBoard() {
    const [isOpen, setIsOpen] = useState(false);
    const [formData, setFormData] = useState({
        name: "",
        contact: "",
        message: "",
        captcha: ""
    });
    const [captcha, setCaptcha] = useState(generateCaptcha());
    const [isSubmitting, setIsSubmitting] = useState(false);
    const [submitStatus, setSubmitStatus] = useState(null);
    const [dailyCount, setDailyCount] = useState(0);
    const lastSubmitTime = useRef(0);

    useEffect(() => {
        setDailyCount(getDailyCount());
    }, []);

    const refreshCaptcha = useCallback(() => {
        setCaptcha(generateCaptcha());
        setFormData(prev => ({ ...prev, captcha: "" }));
    }, []);

    const handleInputChange = (e) => {
        const { name, value } = e.target;
        setFormData(prev => ({
            ...prev,
            [name]: value
        }));
    };

    const handleSubmit = async (e) => {
        e.preventDefault();

        // 节流检查
        const now = Date.now();
        if (now - lastSubmitTime.current < THROTTLE_TIME) {
            setSubmitStatus({ type: "error", message: "请稍后再试" });
            return;
        }
        lastSubmitTime.current = now;

        if (dailyCount >= MAX_DAILY_MESSAGES) {
            setSubmitStatus({ type: "error", message: "今日留言次数已达上限，请明天再试" });
            return;
        }

        if (!formData.name || !formData.contact || !formData.message) {
            setSubmitStatus({ type: "error", message: "请填写姓名、联系方式和留言内容" });
            return;
        }

        const userAnswer = parseInt(formData.captcha, 10);
        if (userAnswer !== captcha.answer) {
            setSubmitStatus({ type: "error", message: "验证码错误，请重新计算" });
            refreshCaptcha();
            return;
        }

        setIsSubmitting(true);
        setSubmitStatus(null);

        try {
            const response = await sendMessage({
                name: formData.name,
                contact: formData.contact,
                message: formData.message
            });

            if (response.data.code === 200) {
                incrementDailyCount();
                setDailyCount(getDailyCount());
                setSubmitStatus({ type: "success", message: "留言发送成功！我会尽快回复你。" });
                setFormData({ name: "", contact: "", message: "", captcha: "" });
                refreshCaptcha();
            } else {
                setSubmitStatus({ type: "error", message: "发送失败，请稍后重试" });
            }
        } catch (error) {
            setSubmitStatus({ type: "error", message: "网络异常，请稍后重试" });
        } finally {
            setIsSubmitting(false);
        }
    };

    const remainingCount = MAX_DAILY_MESSAGES - dailyCount;

    return (
        <div className="message-board-wrapper">
            <button
                className={`message-toggle ${isOpen ? "message-toggle--open" : ""}`}
                onClick={() => setIsOpen(!isOpen)}
                type="button"
            >
                <svg className="message-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
                </svg>
                <span className="message-toggle-text">{isOpen ? "收起" : "留言"}</span>
            </button>

            <div className={`message-board ${isOpen ? "message-board--open" : ""}`}>
                <div className="message-board-header">
                    <h3>给我留言</h3>
                    <button className="message-close" onClick={() => setIsOpen(false)} type="button">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                            <line x1="18" y1="6" x2="6" y2="18" />
                            <line x1="6" y1="6" x2="18" y2="18" />
                        </svg>
                    </button>
                </div>

                <form className="message-form" onSubmit={handleSubmit}>
                    <div className="form-group">
                        <label htmlFor="name">姓名 *</label>
                        <input
                            id="name"
                            name="name"
                            type="text"
                            placeholder="请输入你的姓名"
                            value={formData.name}
                            onChange={handleInputChange}
                        />
                    </div>

                    <div className="form-group">
                        <label htmlFor="contact">联系方式 *</label>
                        <input
                            id="contact"
                            name="contact"
                            type="text"
                            placeholder="手机号或微信号"
                            value={formData.contact}
                            onChange={handleInputChange}
                        />
                    </div>

                    <div className="form-group">
                        <label htmlFor="message">留言内容 *</label>
                        <textarea
                            id="message"
                            name="message"
                            placeholder="请输入你想说的话..."
                            rows={4}
                            value={formData.message}
                            onChange={handleInputChange}
                        />
                    </div>

                    <div className="form-group form-group--captcha">
                        <label htmlFor="captcha">人机验证 *</label>
                        <div className="captcha-row">
                            <span className="captcha-question">{captcha.question}</span>
                            <input
                                id="captcha"
                                name="captcha"
                                type="text"
                                placeholder="请输入答案"
                                value={formData.captcha}
                                onChange={handleInputChange}
                            />
                            <button className="captcha-refresh" onClick={refreshCaptcha} type="button">
                                <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                                    <path d="M23 4v6h-6M1 20v-6h6" />
                                    <path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15" />
                                </svg>
                            </button>
                        </div>
                    </div>

                    {submitStatus && (
                        <div className={`message-status message-status--${submitStatus.type}`}>
                            {submitStatus.message}
                        </div>
                    )}

                    <button className="message-submit" type="submit" disabled={isSubmitting || dailyCount >= MAX_DAILY_MESSAGES}>
                        {isSubmitting ? "发送中..." : "发送留言"}
                    </button>
                </form>

                <p className="message-tip">
                    今日剩余留言次数：{remainingCount} 次
                </p>
            </div>
        </div>
    );
}

export default MessageBoard;
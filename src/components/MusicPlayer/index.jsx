import React, { useState, useRef, useEffect, useCallback } from "react";
import "./index.scss";
import bgm from "./music/bgm.mp3";

function MusicPlayer() {
    const [isPlaying, setIsPlaying] = useState(false);
    const [isMuted, setIsMuted] = useState(false);
    const [progress, setProgress] = useState(0);
    const audioRef = useRef(null);
    const progressRef = useRef(null);

    useEffect(() => {
        const audio = audioRef.current;
        if (audio) {
            audio.loop = true;
            audio.volume = 0.3;
        }
    }, []);

    useEffect(() => {
        const audio = audioRef.current;
        if (!audio) return;

        const updateProgress = () => {
            if (audio.duration) {
                setProgress((audio.currentTime / audio.duration) * 100);
            }
        };

        audio.addEventListener("timeupdate", updateProgress);
        return () => audio.removeEventListener("timeupdate", updateProgress);
    }, []);

    const togglePlay = useCallback(() => {
        const audio = audioRef.current;
        if (!audio) return;

        if (isPlaying) {
            audio.pause();
        } else {
            audio.play().catch(() => {});
        }
        setIsPlaying(!isPlaying);
    }, [isPlaying]);

    const toggleMute = useCallback(() => {
        const audio = audioRef.current;
        if (!audio) return;
        audio.muted = !isMuted;
        setIsMuted(!isMuted);
    }, [isMuted]);

    const handleProgressClick = (e) => {
        const audio = audioRef.current;
        const bar = progressRef.current;
        if (!audio || !bar || !audio.duration) return;

        const rect = bar.getBoundingClientRect();
        const clickX = e.clientX - rect.left;
        const percentage = clickX / rect.width;
        audio.currentTime = percentage * audio.duration;
        setProgress(percentage * 100);
    };

    return (
        <div className={`music-player ${isPlaying ? "music-player--playing" : ""}`}>
            <audio ref={audioRef} src={bgm} />

            <button
                className="music-btn"
                onClick={togglePlay}
                type="button"
                aria-label={isPlaying ? "暂停音乐" : "播放音乐"}
            >
                {isPlaying ? (
                    <svg viewBox="0 0 24 24" fill="currentColor">
                        <rect x="6" y="4" width="4" height="16" rx="1" />
                        <rect x="14" y="4" width="4" height="16" rx="1" />
                    </svg>
                ) : (
                    <svg viewBox="0 0 24 24" fill="currentColor">
                        <path d="M8 5v14l11-7z" />
                    </svg>
                )}
            </button>

            <div className="music-progress" ref={progressRef} onClick={handleProgressClick}>
                <div className="music-progress-bar" style={{ width: `${progress}%` }} />
            </div>

            <button
                className={`music-btn-mute ${isMuted ? "music-btn-mute--muted" : ""}`}
                onClick={toggleMute}
                type="button"
                aria-label={isMuted ? "取消静音" : "静音"}
            >
                {isMuted ? (
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                        <polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5" />
                        <line x1="23" y1="9" x2="17" y2="15" />
                        <line x1="17" y1="9" x2="23" y2="15" />
                    </svg>
                ) : (
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                        <polygon points="11 5 6 9 2 9 2 15 6 15 11 19 11 5" />
                        <path d="M15.54 8.46a5 5 0 0 1 0 7.07" />
                    </svg>
                )}
            </button>

            {isPlaying && (
                <div className="music-indicator">
                    <span /><span /><span />
                </div>
            )}
        </div>
    );
}

export default MusicPlayer;
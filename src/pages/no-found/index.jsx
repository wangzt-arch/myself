import React from 'react'
import { useNavigate } from "react-router-dom";
import './index.scss'

export default function NoFound() {
    const navigate = useNavigate();
  return (
    <div onClick={()=>navigate("/home")}>
      404
      什么也没有呢,点击返回首页
    </div>
  )
}

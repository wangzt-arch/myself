import React from 'react'
import { useNavigate } from 'react-router-dom';
import Header from '../../components/Header';
import Line from '../../components/Line'
import './index.css'


function Home() {
    const navigate = useNavigate()
    return (
        <div className="home" onClick={() => navigate('/myself/docs')}>
            <Header></Header>
            <div className="home-open">
                <Line></Line>
            </div>
        </div>
    )
}

export default Home
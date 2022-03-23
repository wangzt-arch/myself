import React from 'react'

import Header from '../../components/Header';
import Line from '../../components/Line'
import './index.css'


function Home() {
    return (
        <div className="home">
            <Header></Header>
            <div className="home-open">
                <Line></Line>
            </div>
        </div>
    )
}

export default Home
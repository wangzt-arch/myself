import React from 'react'
import Header from '../../components/Header';
import CubeBox from '../../components/cube-box'
import './index.css'


function About() {
    return (
        <div className="about">
            <Header></Header>
            <div className="about-me">
                <div className="about-me_pdf">
                    
                </div>
            </div>
            <div className="cube-box-area">
                <CubeBox></CubeBox>
            </div>
        </div>
    )
}

export default About
import React from 'react'
import Header from '../../components/Header';
import CubeBox from '../../components/cube-box'
import './index.css'


function About() {
    return (
        <div className="about">
            <Header></Header>
            <div className="about-me">
                <CubeBox></CubeBox>
            </div>
        </div>
    )
}

export default About
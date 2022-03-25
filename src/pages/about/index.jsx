import React, { useState, useEffect } from 'react'
import Pdf from 'react-pdf-js'
import Header from '../../components/Header';
import CubeBox from '../../components/cube-box'
import isPc from '../../utils'
import me from './pdf/myself.pdf'
import './index.css'


function About() {
    let [page, setPage] = useState(1);
    let [totalPage, setTotalPage] = useState(null);
    let [scale, setScale] = useState(1);
    useEffect(() => {
        console.log(isPc());
        isPc() ? setScale(1) : setScale(0.6)
    },[])
    
    const narrow = () => {
        setScale(scale - 0.1)
    }
    const enlarge = () => {
        setScale(scale + 0.1)
    }
    const onDocumentComplete = (pages) => {
        setTotalPage(pages)
    }
    const nextPage = () => {
        page < totalPage && setPage(page + 1)
    }
    const prevPage = () => {
        page > 1 && setPage(page - 1)
    }
    return (
        <div className="about">
            <Header></Header>
            <div className="about-me">
                <div>
                    <Pdf className="about-me_pdf" file={me} scale={scale} page={page} onDocumentComplete={onDocumentComplete}></Pdf>
                </div>
            </div>
            <div className="pdf-button">
                <button className='pdf-button_prev' onClick={narrow}>缩小</button>
                <button className='pdf-button_next' onClick={enlarge}>放大</button>
                <button className='pdf-button_next' onClick={prevPage}>上一页</button>
                <button className='pdf-button_next' onClick={nextPage}>下一页</button>
            </div>
            <div className="cube-box-area">
                <CubeBox></CubeBox>
            </div>
        </div>
    )
}

export default About
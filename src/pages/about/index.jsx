import React, { useState } from 'react'
import Pdf from 'react-pdf-js'
import Header from '../../components/Header';
import CubeBox from '../../components/cube-box'
import me from './pdf/myself.pdf'
import './index.css'


function About() {
    let [page, setPage] = useState(1);
    let [totalPage, setTotalPage] = useState(null);

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
                <div className="about-me_pdf">
                    <Pdf file={me} page={page} onDocumentComplete={onDocumentComplete}></Pdf>
                </div>
                <div className="pdf-button">
                    <button className='pdf-button_prev' onClick={prevPage}>上一页</button>
                    <button className='pdf-button_next' onClick={nextPage}>下一页</button>
                </div>
            </div>
            <div className="cube-box-area">
                <CubeBox></CubeBox>
            </div>
        </div>
    )
}

export default About
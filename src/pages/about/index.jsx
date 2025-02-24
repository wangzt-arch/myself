import React, { useState, useEffect, useRef } from 'react'
import Header from '../../components/Header';
import CubeBox from '../../components/cube-box'
import Loading from '../../components/Loading2';
import isPc from '../../utils'
import me from './pdf/myself.pdf'
import { usePdf } from '@mikecousins/react-pdf';
import './index.css'


function About() {
  useEffect(() => {
    isPc() ? setScale(1.05) : setScale(0.6)
  }, [])

  const [page, setPage] = useState(1);
  let [scale, setScale] = useState(1);
  const canvasRef = useRef(null);

  const { pdfDocument } = usePdf({
    file: me,
    page,
    scale,
    canvasRef,
  });

  const nextPage = () => {
    page < pdfDocument.numPages && setPage(page + 1)
  }
  const prevPage = () => {
    page > 1 && setPage(page - 1)
  }
  const narrow = () => {
    setScale(scale - 0.1)
  }
  const enlarge = () => {
    setScale(scale + 0.1)
  }



  return (
    <div className="about">
      <Header></Header>
      <div className="about-me">
        <div className='about-me__loading'>
          <Loading show={!pdfDocument}></Loading>
        </div>
        <div>
          <canvas className="about-me_pdf" ref={canvasRef} />
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
import React from 'react'
import { useNavigate } from 'react-router-dom';


function Home() {
    const navigate = useNavigate()
    return (
        <div onClick={() => navigate('/docs')}>Home</div>
    )
}

export default Home
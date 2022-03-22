import { useRouter } from "next/router";
// import Charge from "../components/Charge";

const Home = () => {
    const router = useRouter();
    const handleClick = () => {
        router.push("/second");
    };
    return (
        <div className="home" onClick={handleClick}>
            welcome to here! just start➡️
            {/* <div> <Charge></Charge></div> */}
        </div>
    );
};
export default Home;

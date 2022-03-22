import { useRouter } from "next/router";

const Home = () => {
    const router = useRouter();
    const handleClick = () => {
        router.push("/second");
    };
    return (
        <div className="home" onClick={handleClick}>
            welcome to here! just start➡️
        </div>
    );
};
export default Home;

import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import { SiRos, SiNvidia, SiUnity, SiPython } from 'react-icons/si';
import { MdMemory, MdPrecisionManufacturing, MdScience, MdSpeed } from 'react-icons/md';
import { GiRobotGolem, GiCpu } from 'react-icons/gi';
import { IoHardwareChip } from 'react-icons/io5';
import { FaBrain } from 'react-icons/fa';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className="hero">
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className="hero__cta">
          <Link
            className="button button--primary button--lg"
            to="/docs/intro">
            Start Learning →
          </Link>
        </div>
      </div>
    </header>
  );
}

type FeatureItem = {
  title: string;
  icon: ReactNode;
  description: string;
  className: string;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'ROS 2 Architecture',
    icon: <SiRos />,
    description: 'Master Robot Operating System 2 fundamentals: nodes, topics, services, actions, parameters, and real-time communication patterns for distributed robotics systems.',
    className: 'feature-card--midnight',
  },
  {
    title: 'Gazebo & Unity Simulation',
    icon: <SiUnity />,
    description: 'Build and test robots in high-fidelity 3D environments with physics-based simulation, sensor modeling, and realistic world dynamics before hardware deployment.',
    className: 'feature-card--teal',
  },
  {
    title: 'NVIDIA Isaac Sim',
    icon: <SiNvidia />,
    description: 'Leverage GPU-accelerated photorealistic simulation with ray tracing, AI workflows, synthetic data generation, and domain randomization for robust robot training.',
    className: 'feature-card--emerald',
  },
  {
    title: 'Humanoid Robotics',
    icon: <GiRobotGolem />,
    description: 'Explore bipedal locomotion, whole-body control, balance algorithms, ZMP theory, and human-robot interaction for next-generation humanoid platforms.',
    className: 'feature-card--gunmetal',
  },
  {
    title: 'Kinematics & Dynamics',
    icon: <MdPrecisionManufacturing />,
    description: 'Deep dive into forward/inverse kinematics, Jacobians, DH parameters, dynamic modeling, and trajectory optimization for manipulator control.',
    className: 'feature-card--violet',
  },
  {
    title: 'Vision-Language-Action',
    icon: <FaBrain />,
    description: 'Implement multimodal AI systems combining computer vision, large language models, and robot control for intelligent task execution and human instruction following.',
    className: 'feature-card--cyan',
  },
  {
    title: 'Edge AI & Hardware',
    icon: <IoHardwareChip />,
    description: 'Deploy neural networks on NVIDIA Jetson, optimize inference pipelines, manage power constraints, and build real-time perception systems for autonomous robots.',
    className: 'feature-card--amber',
  },
  {
    title: 'Control Systems',
    icon: <MdSpeed />,
    description: 'Design PID controllers, implement state-space control, model predictive control (MPC), and adaptive algorithms for robust robot stabilization and tracking.',
    className: 'feature-card--rose',
  },
  {
    title: 'Reinforcement Learning',
    icon: <MdScience />,
    description: 'Train robot policies with deep RL, sim-to-real transfer, reward shaping, and policy optimization for locomotion, manipulation, and navigation tasks.',
    className: 'feature-card--midnight',
  },
  {
    title: 'Sensor Fusion',
    icon: <MdMemory />,
    description: 'Combine data from IMUs, cameras, LIDAR, and depth sensors using Kalman filtering, particle filters, and probabilistic algorithms for robust state estimation.',
    className: 'feature-card--teal',
  },
  {
    title: 'Python for Robotics',
    icon: <SiPython />,
    description: 'Master Python libraries for robotics: NumPy for math, OpenCV for vision, PyTorch for ML, rclpy for ROS 2, and scientific computing ecosystems.',
    className: 'feature-card--gunmetal',
  },
  {
    title: 'Real-Time Systems',
    icon: <GiCpu />,
    description: 'Understand real-time constraints, deterministic execution, RTOS concepts, timing analysis, and low-latency control loops for mission-critical robot applications.',
    className: 'feature-card--emerald',
  },
];

function Feature({icon, title, description, className}: FeatureItem) {
  return (
    <div className={clsx('feature-card', className)}>
      <div className="feature-icon">{icon}</div>
      <h3>{title}</h3>
      <p>{description}</p>
    </div>
  );
}

function HomepageFeatures() {
  return (
    <section>
      <div className="features-grid">
        {FeatureList.map((props, idx) => (
          <Feature key={idx} {...props} />
        ))}
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title="Home"
      description="Complete guide to Physical AI, Humanoid Robotics, ROS 2, Gazebo, Unity, and NVIDIA Isaac Sim - from fundamentals to advanced implementations">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}

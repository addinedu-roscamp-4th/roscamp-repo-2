import React from 'react';
import {
  Container,
  Typography,
  Timeline,
  TimelineItem,
  TimelineSeparator,
  TimelineConnector,
  TimelineContent,
  TimelineDot,
  Paper,
  Box,
} from '@mui/material';
import { motion } from 'framer-motion';
import MovieIcon from '@mui/icons-material/Movie';
import CodeIcon from '@mui/icons-material/Code';
import GroupIcon from '@mui/icons-material/Group';

const experiences = [
  {
    title: 'Zara Studio',
    role: 'Founder & Creative Director',
    period: '2020 - Present',
    description: 'Founded and managed a video production company specializing in corporate and institutional content. Led projects for major clients including Korea Hydro & Nuclear Power.',
    icon: <MovieIcon />,
  },
  {
    title: 'Senior Village Platform',
    role: 'Project Lead',
    period: '2021 - Present',
    description: 'Developed and managed an information-sharing platform for seniors, collaborating with nursing homes to create targeted content and improve accessibility.',
    icon: <GroupIcon />,
  },
  {
    title: 'RODI Project',
    role: 'Backend Engineer',
    period: '2022',
    description: 'Developed a semi-automated upload and order management system using Python and RestAPI. Handled both technical documentation and backend implementation.',
    icon: <CodeIcon />,
  },
];

const Experience: React.FC = () => {
  return (
    <Container maxWidth="lg">
      <Box
        component={motion.div}
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8 }}
        sx={{ mt: 8, mb: 4 }}
      >
        <Typography variant="h3" component="h1" gutterBottom align="center">
          Professional Experience
        </Typography>
        <Timeline position="alternate" sx={{ mt: 4 }}>
          {experiences.map((experience, index) => (
            <TimelineItem key={index}>
              <TimelineSeparator>
                <TimelineDot color="primary">
                  {experience.icon}
                </TimelineDot>
                {index < experiences.length - 1 && <TimelineConnector />}
              </TimelineSeparator>
              <TimelineContent>
                <Paper
                  component={motion.div}
                  initial={{ opacity: 0, x: index % 2 === 0 ? 50 : -50 }}
                  animate={{ opacity: 1, x: 0 }}
                  transition={{ duration: 0.5, delay: index * 0.2 }}
                  elevation={3}
                  sx={{ p: 3, backgroundColor: 'background.paper' }}
                >
                  <Typography variant="h6" component="h2">
                    {experience.title}
                  </Typography>
                  <Typography variant="subtitle1" color="primary">
                    {experience.role}
                  </Typography>
                  <Typography variant="subtitle2" color="text.secondary">
                    {experience.period}
                  </Typography>
                  <Typography variant="body2" sx={{ mt: 1 }}>
                    {experience.description}
                  </Typography>
                </Paper>
              </TimelineContent>
            </TimelineItem>
          ))}
        </Timeline>
      </Box>
    </Container>
  );
};

export default Experience; 
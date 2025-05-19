import React from 'react';
import {
  Container,
  Typography,
  Grid,
  Card,
  CardContent,
  CardMedia,
  Box,
} from '@mui/material';
import { motion } from 'framer-motion';

const projects = [
  {
    title: 'RODI Project',
    description: 'Semi-automated upload and order management system developed using Python and RestAPI. Contributed as a backend engineer, handling both proposal writing and backend development.',
    image: '/placeholder.jpg',
  },
  {
    title: 'Korea Hydro & Nuclear Power Animation',
    description: 'Security education animation project for Korea Hydro & Nuclear Power, demonstrating our capability in creating educational content for large corporations.',
    image: '/placeholder.jpg',
  },
  {
    title: 'Senior Village Platform',
    description: 'Information sharing platform for seniors, featuring collaborative content creation with nursing homes and senior-focused multimedia content.',
    image: '/placeholder.jpg',
  },
  {
    title: 'Environmental Department Event Display',
    description: 'Created dynamic video content for environmental department event displays, showcasing our expertise in event-specific multimedia production.',
    image: '/placeholder.jpg',
  },
];

const Projects: React.FC = () => {
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
          Our Projects
        </Typography>
        <Grid container spacing={4} sx={{ mt: 2 }}>
          {projects.map((project, index) => (
            <Grid
              item
              xs={12}
              md={6}
              key={index}
              component={motion.div}
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, delay: index * 0.1 }}
            >
              <Card
                sx={{
                  height: '100%',
                  display: 'flex',
                  flexDirection: 'column',
                  transition: '0.3s',
                  '&:hover': {
                    transform: 'translateY(-8px)',
                    boxShadow: 6,
                  },
                }}
              >
                <CardMedia
                  component="img"
                  height="200"
                  image={project.image}
                  alt={project.title}
                  sx={{ objectFit: 'cover' }}
                />
                <CardContent>
                  <Typography gutterBottom variant="h5" component="h2">
                    {project.title}
                  </Typography>
                  <Typography variant="body2" color="text.secondary">
                    {project.description}
                  </Typography>
                </CardContent>
              </Card>
            </Grid>
          ))}
        </Grid>
      </Box>
    </Container>
  );
};

export default Projects; 
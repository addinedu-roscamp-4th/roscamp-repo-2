import React from 'react';
import { Container, Typography, Box, Paper } from '@mui/material';
import { motion } from 'framer-motion';

const Home: React.FC = () => {
  return (
    <Container maxWidth="lg">
      <Box
        component={motion.div}
        initial={{ opacity: 0, y: 20 }}
        animate={{ opacity: 1, y: 0 }}
        transition={{ duration: 0.8 }}
        sx={{
          mt: 8,
          display: 'flex',
          flexDirection: 'column',
          alignItems: 'center',
          gap: 4,
        }}
      >
        <Typography variant="h2" component="h1" gutterBottom align="center">
          Welcome to Zara Studio
        </Typography>
        <Paper
          elevation={3}
          sx={{
            p: 4,
            maxWidth: 800,
            width: '100%',
            backgroundColor: 'background.paper',
          }}
        >
          <Typography variant="h5" gutterBottom>
            About Us
          </Typography>
          <Typography variant="body1" paragraph>
            Zara Studio is a creative powerhouse specializing in video content production
            for public institutions and corporations. We've successfully delivered projects
            like security education animations for Korea Hydro & Nuclear Power and various
            other multimedia content.
          </Typography>
          <Typography variant="body1" paragraph>
            Our expertise extends to managing Senior Village, an information-sharing
            platform for seniors, where we collaborate with nursing homes to create
            and curate senior-focused content.
          </Typography>
          <Typography variant="body1">
            With a track record of successful projects and a commitment to quality,
            we continue to push the boundaries of creative content production.
          </Typography>
        </Paper>
      </Box>
    </Container>
  );
};

export default Home; 
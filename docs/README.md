# Robotic Throwing & Catching - Project Website

A modern, responsive website showcasing our CS 639 final project on robotic throwing and catching with UR5e arms.

## 🚀 Quick Start

Simply open `index.html` in your browser:

```bash
# Navigate to website directory
cd website

# Open in browser (Windows)
start index.html

# Or just double-click index.html
```

## 📁 Project Structure

```
website/
├── index.html          # Main HTML file
├── css/
│   └── styles.css      # All styles and responsive design
├── js/
│   └── script.js       # Interactive features and animations
├── assets/
│   ├── images/         # Screenshots, diagrams, team photos
│   ├── videos/         # Local video files (optional)
│   └── graphs/         # Training graphs and visualizations
└── README.md           # This file
```

## 🎯 Features

### Implemented Features

- ✅ Smooth scroll navigation
- ✅ Responsive mobile menu
- ✅ Animated metric counters
- ✅ Fade-in animations on scroll
- ✅ Active navigation highlighting
- ✅ Code block copy functionality
- ✅ Scroll-to-top button
- ✅ Automatic graph loading
- ✅ YouTube video embedding
- ✅ Fully responsive design

### Color Scheme

The website uses a modern dark theme with these primary colors:

- Primary: `#6366f1` (Indigo)
- Secondary: `#ec4899` (Pink)
- Accent: `#14b8a6` (Teal)

To change colors, edit the CSS variables in `css/styles.css`:

```css
:root {
    --primary: #6366f1;
    --secondary: #ec4899;
    --accent: #14b8a6;
    /* ... */
}
```

## 📱 Responsive Design

The website is fully responsive and optimized for:

- Desktop (1200px+)
- Tablet (768px - 1024px)
- Mobile (320px - 768px)

## 🐛 Troubleshooting

**Graphs not showing?**

- Ensure image files are in `assets/graphs/` with correct filenames
- Check browser console for 404 errors
- Verify image file extensions (.png, .jpg)

**Video not loading?**

- Make sure you've replaced `VIDEO_ID` with actual YouTube video ID
- Check that the video is public or unlisted (not private)

**Mobile menu not working?**

- Clear browser cache
- Check that `js/script.js` is loaded correctly

## 📄 License

This website template is part of the CS 639 final project and can be freely used and modified.

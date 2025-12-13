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

## 🎨 Customization Guide

### 1. Adding Your YouTube Demo Video

In `index.html`, find the demo section and replace `VIDEO_ID` with your actual YouTube video ID:

```html
<!-- Around line 380 -->
<iframe 
    src="https://www.youtube.com/embed/YOUR_VIDEO_ID" 
    title="Robot Throwing and Catching Demo" 
    frameborder="0" 
    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" 
    allowfullscreen>
</iframe>
```

### 2. Adding Training Graphs

Place your training visualization images in the `assets/graphs/` folder with these filenames:

- `reward_curve.png` - Reward progression over training
- `success_rate.png` - Success rate over episodes
- `velocity_dist.png` - Distribution of release velocities
- `loss_curves.png` - Training loss curves

The JavaScript will automatically load these images. If files are named differently, update the mapping in `js/script.js` (search for `loadGraphImages`).

### 3. Updating Team Information

In `index.html`, find the team section (around line 430) and update:

```html
<div class="team-card">
    <div class="team-avatar">
        <div class="avatar-placeholder">👤</div>
        <!-- Or add an image: -->
        <!-- <img src="assets/images/team/member1.jpg" alt="Team Member 1"> -->
    </div>
    <h3>Your Name</h3>
    <p class="team-role">Your Focus Area</p>
    <div class="team-links">
        <a href="https://github.com/yourusername" target="_blank" title="GitHub">
            <!-- GitHub icon -->
        </a>
        <a href="https://linkedin.com/in/yourprofile" target="_blank" title="LinkedIn">
            <!-- LinkedIn icon -->
        </a>
    </div>
</div>
```

### 4. Updating Metrics

Update the animated counter values in the results section:

```html
<!-- Find these metric-value elements and change data-target -->
<div class="metric-value" data-target="87">0</div>  <!-- Success Rate % -->
<div class="metric-value" data-target="2.4">0</div> <!-- Avg Distance -->
<div class="metric-value" data-target="3.2">0</div> <!-- Release Velocity -->
<div class="metric-value" data-target="500">0</div> <!-- Training Episodes -->
```

### 5. Updating Performance Table

In the comparison table, update the numbers with your actual results:

```html
<tbody>
    <tr>
        <td><strong>Naive Controller</strong></td>
        <td>45%</td>      <!-- Update these -->
        <td>1.2m</td>
        <td>1.8 m/s</td>
        <td>-</td>
    </tr>
    <tr class="highlight-row">
        <td><strong>RL Learned Controller</strong></td>
        <td>87%</td>      <!-- Update these -->
        <td>2.4m</td>
        <td>3.2 m/s</td>
        <td>~2 hours</td>
    </tr>
</tbody>
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

## 🌐 Deployment Options

### GitHub Pages (Recommended)

1. Push your website folder to your repository
2. Go to Settings > Pages
3. Select your branch and `/website` folder
4. Your site will be live at `https://yourusername.github.io/repo-name/`

### Netlify

1. Drag and drop the `website` folder to [Netlify Drop](https://app.netlify.com/drop)
2. Instant deployment with custom domain support

### Local Development

For live reloading during development:

```bash
# Install a simple HTTP server
npm install -g http-server

# Run in website directory
cd website
http-server -p 8080

# Visit http://localhost:8080
```

## 🔧 Browser Compatibility

- Chrome/Edge 90+
- Firefox 88+
- Safari 14+
- Opera 76+

## 📝 Tips

1. **Image Optimization**: Compress images before adding them to reduce load times
2. **Video Hosting**: YouTube embedding is recommended for video content
3. **Testing**: Test on multiple devices and browsers before deploying
4. **Accessibility**: The site includes semantic HTML and ARIA labels
5. **Performance**: Lazy loading is implemented for images with `data-src` attribute

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

---

Built with ❤️ for Robot Learning

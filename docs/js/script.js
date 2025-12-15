// ===========================
// Smooth Scrolling & Navigation
// ===========================

// Smooth scroll for anchor links
document.querySelectorAll('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function (e) {
        e.preventDefault();
        const target = document.querySelector(this.getAttribute('href'));
        if (target) {
            const navHeight = document.querySelector('#navbar').offsetHeight;
            const targetPosition = target.offsetTop - navHeight;
            
            window.scrollTo({
                top: targetPosition,
                behavior: 'smooth'
            });
            
            // Close mobile menu if open
            const navMenu = document.querySelector('.nav-menu');
            const hamburger = document.querySelector('.hamburger');
            if (navMenu.classList.contains('active')) {
                navMenu.classList.remove('active');
                hamburger.classList.remove('active');
            }
        }
    });
});

// ===========================
// Mobile Navigation Toggle
// ===========================
const hamburger = document.querySelector('.hamburger');
const navMenu = document.querySelector('.nav-menu');

if (hamburger && navMenu) {
    hamburger.addEventListener('click', () => {
        hamburger.classList.toggle('active');
        navMenu.classList.toggle('active');
    });
}

// ===========================
// Navbar Scroll Effect
// ===========================
const navbar = document.querySelector('#navbar');
let lastScroll = 0;

window.addEventListener('scroll', () => {
    const currentScroll = window.pageYOffset;
    
    // Add scrolled class for styling
    if (currentScroll > 50) {
        navbar.classList.add('scrolled');
    } else {
        navbar.classList.remove('scrolled');
    }
    
    lastScroll = currentScroll;
});

// ===========================
// Active Navigation Link
// ===========================
const sections = document.querySelectorAll('section[id]');
const navLinks = document.querySelectorAll('.nav-link');

function setActiveNav() {
    const scrollPosition = window.pageYOffset + 100;
    
    sections.forEach(section => {
        const sectionTop = section.offsetTop;
        const sectionHeight = section.offsetHeight;
        const sectionId = section.getAttribute('id');
        
        if (scrollPosition >= sectionTop && scrollPosition < sectionTop + sectionHeight) {
            navLinks.forEach(link => {
                link.classList.remove('active');
                if (link.getAttribute('href') === `#${sectionId}`) {
                    link.classList.add('active');
                }
            });
        }
    });
}

window.addEventListener('scroll', setActiveNav);
window.addEventListener('load', setActiveNav);

// ===========================
// Animated Counter for Metrics
// ===========================
function animateCounter(element) {
    const target = parseFloat(element.getAttribute('data-target'));
    const duration = 2000; // 2 seconds
    const increment = target / (duration / 16); // 60fps
    let current = 0;
    
    const updateCounter = () => {
        current += increment;
        if (current < target) {
            // Check if decimal or integer
            if (target % 1 !== 0) {
                element.textContent = current.toFixed(1);
            } else {
                element.textContent = Math.floor(current);
            }
            requestAnimationFrame(updateCounter);
        } else {
            if (target % 1 !== 0) {
                element.textContent = target.toFixed(1);
            } else {
                element.textContent = target;
            }
        }
    };
    
    updateCounter();
}

// Intersection Observer for counter animation
const observerOptions = {
    threshold: 0.5,
    rootMargin: '0px'
};

const counterObserver = new IntersectionObserver((entries) => {
    entries.forEach(entry => {
        if (entry.isIntersecting && !entry.target.classList.contains('counted')) {
            animateCounter(entry.target);
            entry.target.classList.add('counted');
        }
    });
}, observerOptions);

// Observe all metric values
document.querySelectorAll('.metric-value').forEach(counter => {
    counterObserver.observe(counter);
});

// ===========================
// Fade-in Animation on Scroll
// ===========================
const fadeElements = document.querySelectorAll('.feature-card, .approach-block, .graph-card, .team-card');

const fadeObserver = new IntersectionObserver((entries) => {
    entries.forEach((entry, index) => {
        if (entry.isIntersecting) {
            setTimeout(() => {
                entry.target.style.opacity = '1';
                entry.target.style.transform = 'translateY(0)';
            }, index * 100);
        }
    });
}, {
    threshold: 0.1,
    rootMargin: '0px'
});

fadeElements.forEach(element => {
    element.style.opacity = '0';
    element.style.transform = 'translateY(30px)';
    element.style.transition = 'opacity 0.6s ease, transform 0.6s ease';
    fadeObserver.observe(element);
});

// ===========================
// Dynamic Graph Loading
// ===========================
function loadGraphImages() {
    const graphCards = document.querySelectorAll('.graph-card');
    
    graphCards.forEach(card => {
        const placeholder = card.querySelector('.graph-placeholder');
        if (placeholder) {
            const title = card.querySelector('h4').textContent.toLowerCase();
            let imageName = '';
            
            // Map titles to image filenames
            if (title.includes('reward')) {
                imageName = 'reward_curve.png';
            } else if (title.includes('success')) {
                imageName = 'success_rate.png';
            } else if (title.includes('velocity')) {
                imageName = 'velocity_dist.png';
            } else if (title.includes('loss')) {
                imageName = 'loss_curves.png';
            }
            
            if (imageName) {
                const img = new Image();
                img.src = `assets/graphs/${imageName}`;
                img.alt = title;
                img.style.width = '100%';
                img.style.height = 'auto';
                img.style.borderRadius = '8px';
                
                img.onload = () => {
                    placeholder.replaceWith(img);
                };
                
                // If image fails to load, keep placeholder
                img.onerror = () => {
                    console.log(`Image not found: ${imageName}`);
                };
            }
        }
    });
}

// Call on page load
window.addEventListener('load', loadGraphImages);

// ===========================
// Scroll to Top Button (Optional)
// ===========================
function createScrollToTop() {
    const button = document.createElement('button');
    button.innerHTML = '↑';
    button.className = 'scroll-to-top';
    button.style.cssText = `
        position: fixed;
        bottom: 2rem;
        right: 2rem;
        width: 50px;
        height: 50px;
        border-radius: 50%;
        background: linear-gradient(135deg, var(--primary) 0%, var(--primary-dark) 100%);
        color: white;
        border: none;
        font-size: 1.5rem;
        cursor: pointer;
        opacity: 0;
        visibility: hidden;
        transition: all 0.3s ease;
        z-index: 999;
        box-shadow: 0 4px 15px rgba(99, 102, 241, 0.4);
    `;
    
    document.body.appendChild(button);
    
    window.addEventListener('scroll', () => {
        if (window.pageYOffset > 300) {
            button.style.opacity = '1';
            button.style.visibility = 'visible';
        } else {
            button.style.opacity = '0';
            button.style.visibility = 'hidden';
        }
    });
    
    button.addEventListener('click', () => {
        window.scrollTo({
            top: 0,
            behavior: 'smooth'
        });
    });
    
    button.addEventListener('mouseenter', () => {
        button.style.transform = 'translateY(-5px)';
    });
    
    button.addEventListener('mouseleave', () => {
        button.style.transform = 'translateY(0)';
    });
}

createScrollToTop();

// ===========================
// Video Placeholder Handler
// ===========================
function handleVideoPlaceholder() {
    const videoWrapper = document.querySelector('.video-wrapper');
    const placeholder = document.querySelector('.video-placeholder');
    const iframe = document.querySelector('.video-wrapper iframe');
    
    if (placeholder && iframe) {
        // Check if iframe has a valid src (not just VIDEO_ID placeholder)
        const iframeSrc = iframe.getAttribute('src');
        if (iframeSrc && !iframeSrc.includes('VIDEO_ID')) {
            placeholder.style.display = 'none';
            iframe.style.display = 'block';
        } else {
            placeholder.style.display = 'flex';
            if (iframe) iframe.style.display = 'none';
        }
    }
}

window.addEventListener('load', handleVideoPlaceholder);

// ===========================
// Copy Code Block Functionality
// ===========================
document.querySelectorAll('.code-block').forEach(block => {
    const copyButton = document.createElement('button');
    copyButton.textContent = 'Copy';
    copyButton.className = 'copy-code-btn';
    copyButton.style.cssText = `
        position: absolute;
        top: 0.75rem;
        right: 1.5rem;
        padding: 0.25rem 0.75rem;
        background: rgba(99, 102, 241, 0.2);
        color: var(--primary-light);
        border: 1px solid var(--primary);
        border-radius: 6px;
        font-size: 0.75rem;
        cursor: pointer;
        transition: var(--transition);
    `;
    
    const header = block.querySelector('.code-header');
    if (header) {
        header.style.position = 'relative';
        header.appendChild(copyButton);
    }
    
    copyButton.addEventListener('click', async () => {
        const code = block.querySelector('code').textContent;
        try {
            await navigator.clipboard.writeText(code);
            copyButton.textContent = 'Copied!';
            copyButton.style.background = 'rgba(20, 184, 166, 0.2)';
            copyButton.style.borderColor = 'var(--accent)';
            copyButton.style.color = 'var(--accent)';
            
            setTimeout(() => {
                copyButton.textContent = 'Copy';
                copyButton.style.background = 'rgba(99, 102, 241, 0.2)';
                copyButton.style.borderColor = 'var(--primary)';
                copyButton.style.color = 'var(--primary-light)';
            }, 2000);
        } catch (err) {
            console.error('Failed to copy code:', err);
        }
    });
    
    copyButton.addEventListener('mouseenter', () => {
        copyButton.style.background = 'rgba(99, 102, 241, 0.3)';
    });
    
    copyButton.addEventListener('mouseleave', () => {
        if (copyButton.textContent === 'Copy') {
            copyButton.style.background = 'rgba(99, 102, 241, 0.2)';
        }
    });
});

// ===========================
// Initialize Everything
// ===========================
document.addEventListener('DOMContentLoaded', () => {
    console.log('🤖 Robotic Throwing & Catching Website Loaded!');
    
    // Add any additional initialization here
    setActiveNav();
});

// ===========================
// Performance: Lazy Load Images
// ===========================
if ('IntersectionObserver' in window) {
    const imageObserver = new IntersectionObserver((entries, observer) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                const img = entry.target;
                img.src = img.dataset.src;
                img.classList.add('loaded');
                observer.unobserve(img);
            }
        });
    });
    
    document.querySelectorAll('img[data-src]').forEach(img => {
        imageObserver.observe(img);
    });
}

// ===========================
// Console Easter Egg
// ===========================
console.log('%c🤖 Robotic Throwing & Catching', 'font-size: 24px; font-weight: bold; color: #6366f1;');
console.log('%cCS 639 Final Project', 'font-size: 14px; color: #94a3b8;');
console.log('%cBuilt with ❤️ for Robot Learning', 'font-size: 12px; color: #ec4899;');
console.log('%cGitHub: https://github.com/ayuan1114/robo-final-proj', 'font-size: 12px; color: #14b8a6;');

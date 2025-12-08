"""
Cleanup script to remove accumulated trajectory files.
Run this to free up disk space and prevent memory issues.
"""

import os
import glob

def cleanup_trajectories():
    """Remove all .npy trajectory files from the archive directory."""
    archive_dir = os.path.join(os.path.dirname(__file__), 'trajectory_archive')
    
    if not os.path.exists(archive_dir):
        print(f"Archive directory does not exist: {archive_dir}")
        return
    
    # Find all .npy files
    npy_files = glob.glob(os.path.join(archive_dir, '*.npy'))
    
    print(f"Found {len(npy_files)} trajectory files to delete")
    
    deleted = 0
    failed = 0
    
    for fpath in npy_files:
        try:
            os.remove(fpath)
            deleted += 1
        except Exception as e:
            print(f"Failed to delete {fpath}: {e}")
            failed += 1
    
    print(f"Cleanup complete: {deleted} deleted, {failed} failed")
    
    # Calculate space freed (approximate)
    print(f"Approximate disk space freed: {deleted * 0.0007:.2f} MB")

if __name__ == '__main__':
    cleanup_trajectories()

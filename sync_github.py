"""
GitHub Sync Helper for Micromouse Project
Repository: https://github.com/padmanaban1908/micromouse
Directly uploads and synchronizes firmware, simulation, algorithms, and documentation via GitHub REST API.
"""

import os
import sys
import json
import base64
import urllib.request
import urllib.error

GITHUB_API_URL = "https://api.github.com"
DEFAULT_REPO_OWNER = "padmanaban1908"
DEFAULT_REPO_NAME = "micromouse"


def get_files_to_sync(root_dir):
    ignored_patterns = {".git", "__pycache__", ".vscode", ".pio", ".venv", "venv", ".idea", "build"}
    ignored_files = {".env", ".token"}
    files = []
    for dirpath, dirnames, filenames in os.walk(root_dir):
        # Exclude ignored directories
        dirnames[:] = [d for d in dirnames if d not in ignored_patterns]
        for f in filenames:
            if f in ignored_files or f.endswith((".pyc", ".log", ".tmp")):
                continue
            full_path = os.path.join(dirpath, f)
            rel_path = os.path.relpath(full_path, root_dir).replace("\\", "/")
            files.append((rel_path, full_path))
    return sorted(files)


def get_default_branch(token, owner, repo):
    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "User-Agent": "Micromouse-Sync-Agent",
    }
    url = f"{GITHUB_API_URL}/repos/{owner}/{repo}"
    try:
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            return data.get("default_branch", "main")
    except urllib.error.HTTPError as e:
        print(f"⚠️ Could not detect default branch, defaulting to 'main'. Details: {e}")
        return "main"


def verify_repo(token, owner, repo):
    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "User-Agent": "Micromouse-Sync-Agent",
    }
    url = f"{GITHUB_API_URL}/repos/{owner}/{repo}"
    try:
        req = urllib.request.Request(url, headers=headers)
        with urllib.request.urlopen(req) as resp:
            data = json.loads(resp.read().decode("utf-8"))
            print(f"✅ Authenticated successfully! Connected to: {data.get('html_url')}")
            print(f"   Default Branch: {data.get('default_branch', 'main')}")
            return True, data.get("default_branch", "main")
    except urllib.error.HTTPError as e:
        if e.code == 401:
            print("❌ Authentication failed (401 Bad Credentials): Verify your token.")
        elif e.code == 404:
            print(f"❌ Repository '{owner}/{repo}' not found or token lacks 'repo' scope.")
        else:
            print(f"❌ GitHub API Error ({e.code}): {e.read().decode('utf-8')}")
        return False, "main"


def upload_file(token, owner, repo, branch, rel_path, local_path, message="Update micromouse project"):
    headers = {
        "Authorization": f"Bearer {token}",
        "Accept": "application/vnd.github+json",
        "User-Agent": "Micromouse-Sync-Agent",
    }

    with open(local_path, "rb") as f:
        content_bytes = f.read()
        content_b64 = base64.b64encode(content_bytes).decode("utf-8")

    url = f"{GITHUB_API_URL}/repos/{owner}/{repo}/contents/{rel_path}"

    # Check if file exists to fetch sha (for update vs create)
    sha = None
    try:
        check_req = urllib.request.Request(f"{url}?ref={branch}", headers=headers)
        with urllib.request.urlopen(check_req) as resp:
            existing = json.loads(resp.read().decode("utf-8"))
            sha = existing.get("sha")
    except urllib.error.HTTPError:
        pass

    payload = {
        "message": message,
        "content": content_b64,
        "branch": branch,
    }
    if sha:
        payload["sha"] = sha

    put_req = urllib.request.Request(
        url,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="PUT",
    )

    try:
        with urllib.request.urlopen(put_req) as resp:
            status = "Updated" if sha else "Created"
            print(f"  [✓] {status}: {rel_path}")
            return True
    except urllib.error.HTTPError as e:
        print(f"  [✗] Failed to sync {rel_path}: {e.read().decode('utf-8')}")
        return False


def sync_all(token, owner=DEFAULT_REPO_OWNER, repo=DEFAULT_REPO_NAME):
    root_dir = os.path.dirname(os.path.abspath(__file__))

    print("=" * 65)
    print(f"🐭 SYNCING MICROMOUSE PROJECT TO GITHUB")
    print(f"   Target: https://github.com/{owner}/{repo}")
    print("=" * 65)

    ok, branch = verify_repo(token, owner, repo)
    if not ok:
        sys.exit(1)

    files = get_files_to_sync(root_dir)
    print(f"\n📦 Found {len(files)} files to upload across code, ideas, and documentation:\n")

    success_count = 0
    for rel_path, full_path in files:
        if upload_file(token, owner, repo, branch, rel_path, full_path, message=f"Sync {rel_path}"):
            success_count += 1

    print("\n" + "=" * 65)
    print(f"🎉 SYNC COMPLETE: {success_count}/{len(files)} files synced successfully!")
    print(f"🔗 View your repository live at: https://github.com/{owner}/{repo}")
    print("=" * 65)


if __name__ == "__main__":
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if os.path.exists(env_path):
        with open(env_path, "r", encoding="utf-8") as f:
            for line in f:
                if line.strip().startswith("GITHUB_TOKEN="):
                    os.environ["GITHUB_TOKEN"] = line.strip().split("=", 1)[1]

    token = os.getenv("GITHUB_TOKEN")

    if not token and len(sys.argv) > 1:
        token = sys.argv[1].strip()

    if not token:
        print("GitHub token not found. Please provide GITHUB_TOKEN in .env or as CLI argument.")
        sys.exit(1)

    sync_all(token)

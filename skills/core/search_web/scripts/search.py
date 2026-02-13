import json
import os
import sys

import requests

# Configuration variables from environment or with default values
SEARXNG_URL = os.environ.get("SEARXNG_URL", "http://jupiter:9080/search")
SEARCH_ENGINES = os.environ.get("SEARXNG_ENGINES", "google,bing")
SEARCH_LANGUAGE = os.environ.get("SEARXNG_LANGUAGE", "en-EN")
SAFE_SEARCH = int(os.environ.get("SEARXNG_SAFE_SEARCH", "1"))


def perform_search(query):
    # Debug info for the console (appears in the ROS launch terminal)
    print(f"DEBUG: Connecting to SearXNG at: {SEARXNG_URL}", file=sys.stderr)
    print(f"DEBUG: Using engines: {SEARCH_ENGINES} (Language: {SEARCH_LANGUAGE})", file=sys.stderr)

    if not query:
        return {"status": "error", "message": "No query provided."}

    # SearXNG parameter configuration
    params = {
        "q": query,
        "format": "json",
        "engines": SEARCH_ENGINES,
        "language": SEARCH_LANGUAGE,
        "safesearch": SAFE_SEARCH,
    }

    try:
        # Added timeout in case the NAS or SearXNG is busy
        response = requests.get(SEARXNG_URL, params=params, timeout=10)

        if response.status_code != 200:
            return {
                "status": "error",
                "message": f"SearXNG Error {response.status_code}",
                "details": response.text,
            }

        data = response.json()

        results = []
        # SearXNG returns results in the 'results' array
        for item in data.get("results", []):
            results.append(
                {
                    "title": item.get("title"),
                    "snippet": item.get("content"),  # SearXNG uses 'content' for the description
                    "link": item.get("url"),  # SearXNG uses 'url' for the link
                }
            )

        return {"status": "success", "results": results}

    except Exception as e:
        return {"status": "error", "message": f"Search execution failed: {str(e)}"}


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(json.dumps({"status": "error", "message": "Missing search query."}))
        sys.exit(1)

    # Join all arguments into a single search string
    query = " ".join(sys.argv[1:])
    print(json.dumps(perform_search(query)))

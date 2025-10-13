#!/bin/bash

# A robust script to perform a web search using the LangSearch API.
#
# Improvements:
# - API key is read from an environment variable for security.
# - Search query is passed as a command-line argument for reusability.
# - Includes error handling and usage instructions.
# - Uses jq to safely build the JSON payload and pretty-print the output.

# Exit immediately if a command exits with a non-zero status.
set -e
set -o pipefail

export LANGSEARCH_API_KEY='sk-affc43166d1f405b8a41cd782c9dd11c'
# --- Configuration ---
# Your LangSearch API key should be set as an environment variable.
# Example: export LANGSEARCH_API_KEY='your_secret_key'
: "${LANGSEARCH_API_KEY?Error: The LANGSEARCH_API_KEY environment variable is not set.}"

# --- Functions ---
usage() {
  echo "Usage: $0 \"<search query>\""
  echo
  echo "Example: $0 \"what is the capital of France\""
  exit 1
}

# --- Main Script ---
# Check if a query argument was provided
if [ -z "$1" ]; then
  echo "Error: No search query provided." >&2
  usage
fi

QUERY="$1"

# Safely construct the JSON payload using jq
# This prevents issues with special characters in the query
JSON_PAYLOAD=$(jq -n \
                  --arg query "$QUERY" \
                  '{
                      "query": $query,
                      "freshness": "noLimit",
                      "summary": true,
                      "count": 10
                  }')

# Perform the API call and store the response
RESPONSE=$(curl --silent --location 'https://api.langsearch.com/v1/web-search' \
  --header "Authorization: Bearer $LANGSEARCH_API_KEY" \
  --header 'Content-Type: application/json' \
  --data "$JSON_PAYLOAD")

# Check if jq is installed to pretty-print the output, otherwise print as is
if command -v jq &> /dev/null; then
  echo "$RESPONSE" | jq '.'
else
  echo "Warning: 'jq' is not installed. For a more readable output, please install it." >&2
  echo "$RESPONSE"
fi

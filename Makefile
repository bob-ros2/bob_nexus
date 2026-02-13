.PHONY: test lint format clean help

help:
	@echo "Mastermind Quality Suite"
	@echo "  make lint    - Run ruff for static analysis"
	@echo "  make format  - Auto-format code with ruff"
	@echo "  make test    - Run integrity and logic tests"
	@echo "  make clean   - Remove cache files"

lint:
	@echo "[*] Running Linter (Ruff)..."
	python3 -m ruff check .

format:
	@echo "[*] Formatting Code (Ruff)..."
	python3 -m ruff format .

test:
	@echo "[*] Running Integrity Tests (Pytest)..."
	PYTHONPATH=master/core:master/cli python3 -m pytest tests/

clean:
	find . -type d -name "__pycache__" -exec rm -rf {} +
	find . -type d -name ".pytest_cache" -exec rm -rf {} +
	find . -type d -name ".ruff_cache" -exec rm -rf {} +

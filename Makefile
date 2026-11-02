# AutoPathingFTC — no package manager needed; these are thin convenience wrappers.
.PHONY: app install serve clean

app:            ## Build build/AutoPathingFTC.app
	./scripts/build-macos-app.sh

install:        ## Build and copy into /Applications (or $$INSTALL_DIR)
	./scripts/build-macos-app.sh --install

serve:          ## Run the planner in any browser at http://localhost:8000
	cd app/web && python3 -m http.server 8000

clean:
	rm -rf build

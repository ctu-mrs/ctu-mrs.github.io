# Documentation at [https://ctu-mrs.github.io/](https://ctu-mrs.github.io/)

# How to update
1) Create a branch with your changes
2) Once you're done, create a pull request
3) Assign a senior MRS member to check the content and
  - to give you feedback or
  - to merge into `master`.

# Running locally

```bash
cd ctu-mrs.github.io
yarn install
yarn add --frozen-lockfile --non-interactive remark-math@6 rehype-katex@7 @aldridged/docusaurus-plugin-lunr docusaurus-lunr-search
npm run start
```

Open `http://localhost:3000` in your browser. **Note**: no need to restart if you make local changes, `npm` should deploy your changes in real-time.

# Troubleshooting

Cleaning `npm` cache
```bash
npm cache clean --force
rm -rf node_modules package-lock.json
```

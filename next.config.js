module.exports = {
  distDir: "dist",
  exportPathMap: function () {
    return {
      '/': { page: '/static/chunks/pages/index.js' }
    }
  }
};

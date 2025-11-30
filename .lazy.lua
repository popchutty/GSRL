return {
  -- LSP
  {
    "neovim/nvim-lspconfig",
    opts = {
      -- Set Clangd startup arguments
      servers = {
        clangd = {
          cmd = {
            "clangd",
            "--background-index",
            "--clang-tidy",
            "--header-insertion=iwyu",
            "--completion-style=detailed",
            "--function-arg-placeholders",
            "--fallback-style=llvm",
            "--compile-commands-dir=build/Debug",
            "--query-driver=/usr/bin/arm-none-eabi-*",
            "--pch-storage=memory",
          },
        },
      },
      -- Disable virtual text for level below ERROR (Avoid being distracted by clang-tidy warnings)
      diagnostics = {
        virtual_text = {
            severity = { min = vim.diagnostic.severity.ERROR },
        },
      },
    },
  },
  -- Format
  {
    "stevearc/conform.nvim",
    opts = {
      -- Specify formatters
      formatters_by_ft = {
        c = { "clang_format " },
        cpp = { "clang_format " },
      },
    },
  },
}

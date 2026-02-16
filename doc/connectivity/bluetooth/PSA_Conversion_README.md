# PSA Documentation Format Conversion

This directory contains the PSA (Platform Security Architecture) usage documentation in two formats:

## Files

1. **PSA_Usage_in_BLE_Host.md** - Original Markdown format
   - Standard GitHub-flavored markdown
   - Suitable for GitHub, GitLab, and markdown readers
   - File size: ~47KB

2. **PSA_Usage_in_BLE_Host.confluence** - Confluence Wiki Markup format
   - Converted for Atlassian Confluence wiki
   - Uses Confluence-specific syntax
   - File size: ~46KB

## Conversion Details

The Confluence version was automatically converted from the Markdown version with the following transformations:

### Syntax Conversions

| Element | Markdown | Confluence |
|---------|----------|------------|
| **Headers** | `# Title` | `h1. Title` |
| **Bold** | `**text**` | `*text*` |
| **Inline Code** | `` `code` `` | `{{code}}` |
| **Code Blocks** | ` ```language` | `{code:language}` |
| **Links** | `[text](url)` | `[text\|url]` |
| **Lists** | `- item` | `* item` |
| **Numbered** | `1. item` | `# item` |
| **Tables** | `\| col \|` | `\|\|col\|\|` (headers) |
| **Horizontal Rule** | `---` | `----` |

### Special Handling

- **ASCII Diagrams**: Preserved in `{code}` blocks for proper monospace rendering
- **Table Headers**: Converted from `| Header |` to `||Header||`
- **Nested Lists**: Indentation converted to multiple markers (`**` for level 2, etc.)
- **Inline Formatting in Tables**: Preserved through the conversion

## Usage

### For Markdown Readers
Use `PSA_Usage_in_BLE_Host.md` for:
- GitHub/GitLab rendering
- Local markdown editors
- Documentation websites using markdown

### For Confluence
Use `PSA_Usage_in_BLE_Host.confluence` for:
- Atlassian Confluence wiki pages
- Copy-paste directly into Confluence editor (Source Editor mode)
- Confluence Cloud or Server instances

## Importing to Confluence

To import the Confluence file:

1. Open your Confluence page in **edit mode**
2. Click on **Insert** > **Markup**
3. Select **Wiki Markup** 
4. Copy the entire content of `PSA_Usage_in_BLE_Host.confluence`
5. Paste into the Wiki Markup dialog
6. Click **Insert**

Alternatively, use the **Source Editor**:
1. Open page in edit mode
2. Use the **{...}** (Source Editor) option
3. Paste the Confluence wiki markup
4. Preview and save

## Maintaining Both Formats

When updating the documentation:

1. **Update the Markdown version first** (`PSA_Usage_in_BLE_Host.md`)
2. **Re-run the conversion script** to generate updated Confluence version
3. **Review the Confluence output** to ensure formatting is correct
4. **Commit both files** together

## Conversion Script

The conversion was performed using a Python script that handles:
- Header level conversion
- Bold/italic marker transformation
- Link format conversion
- Code block macro wrapping
- Table structure conversion
- List marker transformation

The script can be found in the repository history or can be recreated using the conversion patterns documented above.

## Version Information

- **Document Version**: 1.0
- **Zephyr Commit**: 05b84df99688864910186370334889aebbb1505b
- **Last Updated**: 2026-02-16
- **Conversion Date**: 2026-02-16

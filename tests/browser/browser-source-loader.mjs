import fs from 'node:fs/promises';

/** Load the real dependency into a scoped block for existing VM/data-URL fixtures. */
export async function loadBrowserSource(name) {
    const file = name.endsWith('.js') || name.endsWith('.mjs') ? name : `${name}.js`;
    let source = await fs.readFile(new URL(`../../web/js/${file}`, import.meta.url), 'utf8');
    const imports = [...source.matchAll(/^import\s*\{([^}]+)\}\s*from\s*['"]\.\/low-feature\.mjs(?:\?[^'"]*)?['"];?$/gm)];
    if (!imports.length) return source;
    const dependency = await fs.readFile(new URL('../../web/js/low-feature.mjs', import.meta.url), 'utf8');
    for (const match of imports) {
        source = source.replace(match[0], `const {${match[1]}} = (() => {\n${dependency.replace(/^export /gm, '')}\nreturn {${match[1]}};\n})();`);
    }
    return source;
}

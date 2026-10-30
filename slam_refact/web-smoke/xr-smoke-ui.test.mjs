import assert from 'node:assert/strict';
import test from 'node:test';
import { chromium } from '../../scripts/dev/node_modules/playwright/index.mjs';
import { openDetailsIfClosed } from './xr-smoke-ui.mjs';

test('target controls stay visible in flat, nested and already-open menus without unnecessary clicks', async () => {
  const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true });
  try {
    const page = await browser.newPage();
    for (const nested of [null, false, true]) {
      const controls = '<div id="demo-targets"><label><input type="checkbox" data-name="demo-flat">Flat target</label></div>';
      await page.setContent(`<details id="panel"><summary>Menu</summary>${nested === null ? controls
        : `<details id="advanced" ${nested ? 'open' : ''}><summary>Targets</summary>${controls}</details>`}</details>`);
      await page.evaluate(() => {
        window.summaryClicks = [];
        document.querySelectorAll('summary').forEach((s) => s.addEventListener('click', () => window.summaryClicks.push(s.parentElement.id)));
      });
      await page.locator('#panel > summary').click();
      await openDetailsIfClosed(page.locator('#demo-targets').locator('xpath=ancestor::details[1]'));
      assert.equal(await page.locator('#panel').evaluate((el) => el.open), true, `nested=${nested}: panel remains open`);
      assert.equal(await page.locator('#demo-targets input').isVisible(), true, `nested=${nested}: target control is visible`);
      assert.deepEqual(await page.evaluate(() => window.summaryClicks), nested === false ? ['panel', 'advanced'] : ['panel']);
    }
  } finally {
    await browser.close();
  }
});

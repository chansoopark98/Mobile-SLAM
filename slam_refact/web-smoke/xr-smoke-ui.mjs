export async function openDetailsIfClosed(details) {
  if (!(await details.evaluate((el) => el.open))) await details.locator(':scope > summary').click();
}
